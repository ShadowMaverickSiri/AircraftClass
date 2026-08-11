#pragma once

#include "tacview/Tacview.h"
#include <string>

namespace tacview {

/**
 * @brief 将结构化遥测数据编码为 Tacview ACMI 2.2 文本。
 *
 * 编码器无内部状态，不负责参数校验、网络和文件 I/O。每个函数返回的
 * 字符串都已包含行尾换行符，可直接写入文件或 TCP 数据流。
 */
class AcmiEncoder {
public:
    /** @brief 生成 FileType、FileVersion 和数据源等全局文件头。 */
    static std::string header(const Options& options);
    /** @brief 生成 `#秒数` 时间帧；精确到毫秒。 */
    static std::string frameTime(double time);
    /**
     * @brief 生成对象定义；state 非空时同时携带当前位置与姿态。
     *
     * 携带 state 的形式用于向晚连接客户端发送完整对象快照。
     */
    static std::string objectDefinition(const ObjectInfo& info,
                                        const ObjectState* state = nullptr);
    /** @brief 生成已注册对象的位置与姿态增量。 */
    static std::string objectUpdate(const ObjectState& state);
    /** @brief 生成不影响模型匹配的 ShortName 和 Label 属性。 */
    static std::string objectProperties(ObjectId id, const ObjectProperties& properties);
    /** @brief 生成 `-ID` 对象删除记录。 */
    static std::string objectRemoval(ObjectId id);
    /** @brief 将数值 ID 转成 ACMI 使用的大写十六进制字符串。 */
    static std::string objectId(ObjectId id);
};

} // namespace tacview
