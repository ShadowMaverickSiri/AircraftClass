#include "Tacview.h"
#include <chrono>
#include <thread>

int main() {
    tacview::Options options;
    options.recordingFile = "flight.acmi"; // 不需要文件记录时删除本行

    tacview::Tacview output(options);
    if (!output.start()) return 1;

    output.addObject({1, "Demo Aircraft", "Air+FixedWing", "Blue"});

    for (int i = 0; i < 100; ++i) {
        tacview::ObjectState state;
        state.id = 1;
        state.time = i * 0.1;
        state.longitude = 116.0 + i * 0.00001;
        state.latitude = 40.0;
        state.altitude = 5000.0;
        state.yaw = 90.0;
        output.update(state);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    output.removeObject(1, 10.0);
    output.stop();
}
