#include "tacview/Tacview.h"
#include <chrono>
#include <thread>

int main() {
    // Options 有可直接工作的默认值：监听 42674，服务名为 Tacview Telemetry。
    // recordingFile 非空时，实时数据还会同步保存，便于之后在 Tacview 中回放。
    tacview::Options options;
    options.recordingFile = "flight.acmi"; // 不需要文件记录时删除本行

    tacview::Tacview output(options);
    if (!output.start()) {
        // 实际项目中应记录 lastError()，常见原因是端口已被其他程序占用。
        return 1;
    }

    // 对象必须先注册。ID 在同一次服务运行期间保持唯一，0 不可使用。
    output.addObject({1, "Demo Aircraft", "Air+FixedWing", "Blue"});

    // 示例以 10 Hz 发送 10 秒数据。真实应用只需把数据源转换成 ObjectState；
    // 不需要等待 Tacview 连接，也不需要自己拼接 ACMI 文本。
    for (int i = 0; i < 100; ++i) {
        tacview::ObjectState state;
        state.id = 1;
        state.time = i * 0.1;
        state.longitude = 116.0 + i * 0.00001; // 产生缓慢向东移动的显示效果
        state.latitude = 40.0;
        state.altitude = 5000.0;
        state.yaw = 90.0;
        output.update(state);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // removeObject 会向 Tacview 发送删除记录，并从晚连接快照中移除该对象。
    output.removeObject(1, 10.0);
    // 显式停止会排空发送队列，确保 flight.acmi 包含最后的删除记录。
    output.stop();
}
