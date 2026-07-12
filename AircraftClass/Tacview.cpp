#include "Tacview.h"
#include "AcmiEncoder.h"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

namespace tacview {

class Tacview::Impl {
public:
    explicit Impl(Options value) : options(std::move(value)) {}
    ~Impl() { stop(); }

    bool start();
    void stop();
    bool addObject(const ObjectInfo& object);
    bool update(const ObjectState& state);
    bool removeObject(ObjectId id, double time);

    bool isRunning() const { return running.load(); }
    std::size_t clientCount() const;
    std::string lastError() const;

private:
    struct TrackedObject {
        ObjectInfo info;
        ObjectState state;
        bool hasState = false;
    };

    void acceptLoop();
    void sendLoop();
    void enqueue(std::string message);
    void broadcast(const std::string& message);
    std::string snapshot() const;
    bool sendAll(SOCKET socket, const std::string& message) const;
    bool validState(const ObjectState& state);
    void setError(std::string message);

    Options options;
    SOCKET listenSocket = INVALID_SOCKET;
    std::atomic_bool running{false};
    bool winsockReady = false;
    std::thread acceptThread;
    std::thread sendThread;

    mutable std::mutex clientsMutex;
    std::vector<SOCKET> clients;

    mutable std::mutex objectsMutex;
    std::map<ObjectId, TrackedObject> objects;

    std::mutex queueMutex;
    std::condition_variable queueChanged;
    std::deque<std::string> queue;

    mutable std::mutex errorMutex;
    std::string error;

    std::mutex recordingMutex;
    std::ofstream recording;
};

Tacview::Tacview(Options options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

Tacview::~Tacview() = default;
bool Tacview::start() { return impl_->start(); }
void Tacview::stop() { impl_->stop(); }
bool Tacview::addObject(const ObjectInfo& object) { return impl_->addObject(object); }
bool Tacview::update(const ObjectState& state) { return impl_->update(state); }
bool Tacview::removeObject(ObjectId id, double time) { return impl_->removeObject(id, time); }
bool Tacview::isRunning() const { return impl_->isRunning(); }
std::size_t Tacview::clientCount() const { return impl_->clientCount(); }
std::string Tacview::lastError() const { return impl_->lastError(); }

bool Tacview::Impl::start() {
    if (running.load()) return true;
    if (options.queueCapacity == 0) {
        setError("queueCapacity must be greater than zero");
        return false;
    }

    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        setError("WSAStartup failed");
        return false;
    }
    winsockReady = true;

    listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSocket == INVALID_SOCKET) {
        setError("Unable to create TCP socket");
        stop();
        return false;
    }

    BOOL reuse = TRUE;
    setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(options.port);
    if (bind(listenSocket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR ||
        listen(listenSocket, SOMAXCONN) == SOCKET_ERROR) {
        setError("Unable to bind or listen on port " + std::to_string(options.port));
        stop();
        return false;
    }

    if (!options.recordingFile.empty()) {
        bool opened = false;
        {
            std::lock_guard<std::mutex> lock(recordingMutex);
            recording.open(options.recordingFile, std::ios::out | std::ios::trunc);
            opened = recording.good();
            if (opened) recording << AcmiEncoder::header(options);
        }
        if (!opened) {
            setError("Unable to open recording file: " + options.recordingFile);
            stop();
            return false;
        }
    }

    running.store(true);
    acceptThread = std::thread(&Impl::acceptLoop, this);
    sendThread = std::thread(&Impl::sendLoop, this);
    return true;
}

void Tacview::Impl::stop() {
    running.store(false);
    queueChanged.notify_all();

    if (listenSocket != INVALID_SOCKET) {
        shutdown(listenSocket, SD_BOTH);
        closesocket(listenSocket);
        listenSocket = INVALID_SOCKET;
    }
    if (acceptThread.joinable()) acceptThread.join();
    if (sendThread.joinable()) sendThread.join();

    {
        std::lock_guard<std::mutex> lock(clientsMutex);
        for (SOCKET client : clients) {
            shutdown(client, SD_BOTH);
            closesocket(client);
        }
        clients.clear();
    }
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        queue.clear();
    }
    {
        std::lock_guard<std::mutex> lock(recordingMutex);
        if (recording.is_open()) recording.close();
    }
    if (winsockReady) {
        WSACleanup();
        winsockReady = false;
    }
}

bool Tacview::Impl::addObject(const ObjectInfo& object) {
    if (!running.load()) {
        setError("Tacview server is not running");
        return false;
    }
    if (object.id == 0) {
        setError("Object id 0 is reserved by ACMI");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(objectsMutex);
        if (objects.find(object.id) != objects.end()) {
            setError("Object id is already registered");
            return false;
        }
        objects.emplace(object.id, TrackedObject{object, {}, false});
    }
    enqueue(AcmiEncoder::objectDefinition(object));
    return true;
}

bool Tacview::Impl::update(const ObjectState& state) {
    if (!running.load()) {
        setError("Tacview server is not running");
        return false;
    }
    if (!validState(state)) return false;
    {
        std::lock_guard<std::mutex> lock(objectsMutex);
        auto found = objects.find(state.id);
        if (found == objects.end()) {
            setError("Object must be added before update");
            return false;
        }
        found->second.state = state;
        found->second.hasState = true;
    }
    enqueue(AcmiEncoder::frameTime(state.time) + AcmiEncoder::objectUpdate(state));
    return true;
}

bool Tacview::Impl::removeObject(ObjectId id, double time) {
    if (!running.load()) {
        setError("Tacview server is not running");
        return false;
    }
    if (!std::isfinite(time) || time < 0.0) {
        setError("Removal time must be a finite non-negative value");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(objectsMutex);
        if (objects.erase(id) == 0) {
            setError("Object id is not registered");
            return false;
        }
    }
    enqueue(AcmiEncoder::frameTime(time) + AcmiEncoder::objectRemoval(id));
    return true;
}

bool Tacview::Impl::validState(const ObjectState& state) {
    const double values[] = {state.time, state.longitude, state.latitude,
                             state.altitude, state.roll, state.pitch, state.yaw};
    for (double value : values) {
        if (!std::isfinite(value)) {
            setError("Object state contains NaN or infinity");
            return false;
        }
    }
    if (state.id == 0 || state.time < 0.0 || state.longitude < -180.0 ||
        state.longitude > 180.0 || state.latitude < -90.0 || state.latitude > 90.0) {
        setError("Object state contains an invalid id, time, longitude, or latitude");
        return false;
    }
    return true;
}

void Tacview::Impl::enqueue(std::string message) {
    if (!running.load()) {
        setError("Tacview server is not running");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (queue.size() >= options.queueCapacity) queue.pop_front();
        queue.push_back(std::move(message));
    }
    queueChanged.notify_one();
}

void Tacview::Impl::sendLoop() {
    while (true) {
        std::string message;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueChanged.wait(lock, [this] { return !queue.empty() || !running.load(); });
            if (queue.empty() && !running.load()) break;
            message = std::move(queue.front());
            queue.pop_front();
        }
        {
            std::lock_guard<std::mutex> lock(recordingMutex);
            if (recording.is_open()) recording << message << std::flush;
        }
        broadcast(message);
    }
}

void Tacview::Impl::acceptLoop() {
    while (running.load()) {
        SOCKET client = accept(listenSocket, nullptr, nullptr);
        if (client == INVALID_SOCKET) {
            if (running.load()) setError("Accepting a Tacview client failed");
            break;
        }

        DWORD timeout = 2000;
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        std::string handshake = "XtraLib.Stream.0\nTacview.RealTimeTelemetry.0\n" +
                                options.serverName + "\n";
        handshake.push_back('\0');
        // 与广播共用同一把锁，保证“快照之后的第一条增量”不会丢失。
        std::lock_guard<std::mutex> lock(clientsMutex);
        const std::string initialData = AcmiEncoder::header(options) + snapshot();
        if (sendAll(client, handshake) && sendAll(client, initialData)) {
            clients.push_back(client);
        } else {
            closesocket(client);
        }
    }
}

std::string Tacview::Impl::snapshot() const {
    std::lock_guard<std::mutex> lock(objectsMutex);
    std::string result;
    double newestTime = 0.0;
    bool hasState = false;
    for (const auto& pair : objects) {
        if (pair.second.hasState) {
            newestTime = (std::max)(newestTime, pair.second.state.time);
            hasState = true;
        }
    }
    if (hasState) result += AcmiEncoder::frameTime(newestTime);
    for (const auto& pair : objects) {
        const TrackedObject& object = pair.second;
        result += AcmiEncoder::objectDefinition(
            object.info, object.hasState ? &object.state : nullptr);
    }
    return result;
}

void Tacview::Impl::broadcast(const std::string& message) {
    std::lock_guard<std::mutex> lock(clientsMutex);
    for (auto it = clients.begin(); it != clients.end();) {
        if (!sendAll(*it, message)) {
            closesocket(*it);
            it = clients.erase(it);
        } else {
            ++it;
        }
    }
}

bool Tacview::Impl::sendAll(SOCKET socket, const std::string& message) const {
    std::size_t sent = 0;
    while (sent < message.size()) {
        const int remaining = static_cast<int>(message.size() - sent);
        const int count = send(socket, message.data() + sent, remaining, 0);
        if (count == SOCKET_ERROR || count == 0) return false;
        sent += static_cast<std::size_t>(count);
    }
    return true;
}

std::size_t Tacview::Impl::clientCount() const {
    std::lock_guard<std::mutex> lock(clientsMutex);
    return clients.size();
}

void Tacview::Impl::setError(std::string message) {
    std::lock_guard<std::mutex> lock(errorMutex);
    error = std::move(message);
}

std::string Tacview::Impl::lastError() const {
    std::lock_guard<std::mutex> lock(errorMutex);
    return error;
}

} // namespace tacview
