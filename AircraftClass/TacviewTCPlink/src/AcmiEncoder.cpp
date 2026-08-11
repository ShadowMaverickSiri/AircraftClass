#include "tacview/AcmiEncoder.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace tacview {
namespace {

std::string safeText(std::string value) {
    // ACMI 属性以逗号和换行分隔。本模块的简洁接口暂不实现转义语法，
    // 因此把分隔符替换为空格，确保用户文本不会截断或伪造下一条记录。
    for (char& ch : value) {
        if (ch == ',' || ch == '\n' || ch == '\r') ch = ' ';
    }
    return value;
}

void appendTransform(std::ostringstream& out, const ObjectState& state) {
    // ACMI T 字段顺序固定为：经度|纬度|高度|滚转|俯仰|偏航。
    // 经纬度保留 7 位小数以满足轨迹显示精度，其余量保留 2 位小数。
    out << ",T=" << std::fixed << std::setprecision(7)
        << state.longitude << '|' << state.latitude << '|'
        << std::setprecision(2) << state.altitude << '|'
        << state.roll << '|' << state.pitch << '|' << state.yaw;
}

} // namespace

std::string AcmiEncoder::header(const Options& options) {
    // ID 0 表示全局属性，不属于任何具体的遥测对象。
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
    // 新客户端连接时把定义和最后状态合在一行，可立即创建并定位对象。
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

std::string AcmiEncoder::objectProperties(ObjectId id,
                                          const ObjectProperties& properties) {
    std::ostringstream out;
    out << objectId(id);
    if (!properties.shortName.empty()) {
        out << ",ShortName=" << safeText(properties.shortName);
    }
    if (!properties.label.empty()) {
        out << ",Label=" << safeText(properties.label);
    }
    out << '\n';
    return out.str();
}

std::string AcmiEncoder::objectRemoval(ObjectId id) {
    return "-" + objectId(id) + "\n";
}

std::string AcmiEncoder::objectId(ObjectId id) {
    // Tacview 接受十六进制对象 ID；不添加 0x 前缀。
    std::ostringstream out;
    out << std::uppercase << std::hex << id;
    return out.str();
}

} // namespace tacview
