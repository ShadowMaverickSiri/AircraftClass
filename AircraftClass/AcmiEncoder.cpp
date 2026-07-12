#include "AcmiEncoder.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace tacview {
namespace {

std::string safeText(std::string value) {
    // ACMI 属性以逗号和换行分隔；替换这些字符可保持一行一条记录。
    for (char& ch : value) {
        if (ch == ',' || ch == '\n' || ch == '\r') ch = ' ';
    }
    return value;
}

void appendTransform(std::ostringstream& out, const ObjectState& state) {
    out << ",T=" << std::fixed << std::setprecision(7)
        << state.longitude << '|' << state.latitude << '|'
        << std::setprecision(2) << state.altitude << '|'
        << state.roll << '|' << state.pitch << '|' << state.yaw;
}

} // namespace

std::string AcmiEncoder::header(const Options& options) {
    std::ostringstream out;
    out << "FileType=text/acmi/tacview\n"
        << "FileVersion=2.2\n"
        << "0,DataSource=" << safeText(options.dataSource) << "\n"
        << "0,DataRecorder=TacviewTelemetryModule\n";
    return out.str();
}

std::string AcmiEncoder::frameTime(double time) {
    std::ostringstream out;
    out << '#' << std::fixed << std::setprecision(3) << time << '\n';
    return out.str();
}

std::string AcmiEncoder::objectDefinition(const ObjectInfo& info,
                                          const ObjectState* state) {
    std::ostringstream out;
    out << objectId(info.id)
        << ",Type=" << safeText(info.type)
        << ",Name=" << safeText(info.name)
        << ",Color=" << safeText(info.color);
    if (state) appendTransform(out, *state);
    out << '\n';
    return out.str();
}

std::string AcmiEncoder::objectUpdate(const ObjectState& state) {
    std::ostringstream out;
    out << objectId(state.id);
    appendTransform(out, state);
    out << '\n';
    return out.str();
}

std::string AcmiEncoder::objectRemoval(ObjectId id) {
    return "-" + objectId(id) + "\n";
}

std::string AcmiEncoder::objectId(ObjectId id) {
    std::ostringstream out;
    out << std::uppercase << std::hex << id;
    return out.str();
}

} // namespace tacview
