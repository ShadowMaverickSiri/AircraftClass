#ifndef ACMI_H
#define ACMI_H

#include <cstring>
#include <cstdint>
#include <cstdio>

// 对象ID的最大值
const uint64_t MAX_NUM_OBJS = 0xFFFFFFFFFFFFFFFE;

// ============================================================================
// ACMI 格式生成器
// ============================================================================
//
// 功能说明：
// ---------
// ACMI (Universal Flight Data Format) 是 Tacview 软件使用的通用飞行数据
// 交换格式。这个类用于生成符合 ACMI 2.2 标准的数据格式，可以记录和
// 回放飞行仿真数据。
//
// ACMI 文件结构：
// --------------
// 1. 文件头（Header）：
//    - FileType: 文件类型标识
//    - FileVersion: 版本号
//    - ReferenceTime: 参考时间
//    - DataSource/DataRecorder: 数据来源和记录设备信息
//    - Author/Title/Comments: 作者、标题、注释
//    - ReferenceLongitude/Latitude: 参考坐标
//
// 2. 时间帧（Frame）：
//    - 以 # 开头，后面跟时间戳
//    - 例如：#0.1 表示相对时间 0.1 秒
//
// 3. 对象定义（Object Definition）：
//    - 格式：对象ID,Type=类型,Name=名称,Color=颜色,T=位置
//    - 例如：3EC,Type=Air+FixedWing,Name=Su-27,Color=Red,T=116.0|39.0|9000.0
//
// 4. 状态更新（State Update）：
//    - 格式：对象ID,T=经度|纬度|高度|滚转|俯仰|偏航
//    - 例如：3EC,T=116.000123|39.000456|9050.5|10.2|-5.3|180.0
//
// 数据字段说明：
// ------------
// - 经度：6位小数精度
// - 纬度：6位小数精度
// - 高度：1位小数精度（单位：米）
// - 滚转角：1位小数精度（单位：度）
// - 俯仰角：1位小数精度（单位：度）
// - 偏航角：1位小数精度（单位：度）
//
// 使用示例：
// --------
// acmi generator;
// generator.begin();
// generator.setFileType("text/acmi/tacview");
// generator.setFileVersion("2.2");
// generator.setDataSource("AircraftClass Simulation");
// generator.createHeader();
// // ... 发送 header ...
//
// generator.setTimeDelta(0.1);
// generator.setObjectID(0x3EC);
// generator.setOrientation(0.0, 0.0, 180.0);
// generator.setAltitude(9000.0);
// // ... 生成并发送帧数据 ...
//
// ============================================================================

class acmi {
public:
    // 生成的文件头内容（最多1000字符）
    char header[1000];

    // 生成的帧数据内容（最多1000字符）
    char frame[1000];

    // 额外的属性值（最多1000字符）
    char extraVals[1000];

    // 初始化所有内部状态，重置为默认值
    void begin();

    // ========================================================================
    // 文件元数据设置方法
    // ========================================================================

    // 设置文件类型
    // 标准值：text/acmi/tacview
    void setFileType(const char _fileType[]);

    // 设置文件版本
    // 标准值：2.2
    void setFileVersion(const char _fileVersion[]);

    // 设置参考时间
    // 参数：年、月、日、时、分、秒
    // 格式：ISO 8601 (2011-06-02T05:00:00Z)
    void setReferenceTime(const int& _year = 0, const int& _month = 0, const int& _day = 0,
                           const int& _hour = 0, const int& _min = 0, const float& _second = 0);

    // 设置数据源标识
    // 例如：AircraftClass Simulation
    void setDataSource(const char _dataSource[]);

    // 设置数据记录设备标识
    // 例如：KinematicManeuverSystem
    void setDataRecorder(const char _dataRecorder[]);

    // 设置作者信息
    void setAuthor(const char _author[]);

    // 设置标题
    void setTitle(const char _title[]);

    // 设置注释信息
    void setComments(const char _comments[]);

    // 设置参考经度（用于坐标系转换）
    // 注意：参数名为 _lat 但实际设置的是经度（原代码命名问题）
    void setReferenceLongitude(const float& _lat = 0);

    // 设置参考纬度（用于坐标系转换）
    // 注意：参数名为 _lon 但实际设置的是纬度（原代码命名问题）
    void setReferenceLatitude(const float& _lon = 0);

    // 生成文件头内容
    // 调用前需要先设置所有元数据
    void createHeader();

    // ========================================================================
    // 对象属性设置方法
    // ========================================================================

    // 设置时间增量（相对上一帧的时间间隔，单位：秒）
    void setTimeDelta(const float& _timeDelta);

    // 设置对象ID（十六进制数）
    // 例如：0x3EC
    void setObjectID(const uint64_t& _objectID);

    // 设置对象名称
    // 例如：Su-27, F-16, etc.
    void setObjectName(const char _objectName[]);

    // 设置对象位置（经纬度）
    // 参数：纬度、经度
    void setPosition(const float& _lat, const float& _lon);

    // 设置对象位置（XY坐标）
    // 参数：X坐标、Y坐标
    void setPositionXY(const float& _x, const float& _y);

    // 设置对象姿态角
    // 参数：俯仰角、滚转角、偏航角（单位：度）
    void setOrientation(const float& _pitch, const float& _roll, const float& _yaw);

    // 设置航向角
    // 参数：航向角（单位：度）
    void setHeading(const float& _heading);

    // 设置海拔高度
    // 参数：高度值（单位：米）
    void setAltitude(const float& _altitude = 0);

    // 设置显示颜色
    // 常用值：Red, Blue, Green, Orange, etc.
    void setColor(const char _color[]);

    // 设置自定义属性值
    // 参数：属性名称、属性值
    // 例如：setValue("Pilot", "John Doe");
    void setValue(const char _valueName[], const char _value[]);

    // 创建对象条目（生成帧数据）
    // 参数：_header - 输出缓冲区（未使用，原代码未完成）
    void createEntry(char _header[]);

private:
    // 文件头格式模板（printf 格式字符串）
    static const char headerTemplate[];

    // 帧数据格式模板（printf 格式字符串）
    static const char frameTemplate[];

    // ========================================================================
    // 文件元数据存储
    // ========================================================================
    char fileType[40];       // 文件类型
    char fileVersion[10];    // 文件版本
    char referenceTime[40];  // 参考时间字符串
    char dataSource[40];     // 数据源
    char dataRecorder[40];   // 数据记录器
    char author[40];         // 作者
    char title[40];          // 标题
    char comments[40];       // 注释
    char latStr[40];         // 纬度字符串
    char lonStr[40];         // 经度字符串

    int year;                // 年
    int month;               // 月
    int day;                 // 日
    int hour;                // 时
    int min;                 // 分
    float second;            // 秒
    float refLat;            // 参考纬度
    float refLon;            // 参考经度

    // ========================================================================
    // 对象状态存储
    // ========================================================================
    float timeDelta;         // 时间增量
    uint64_t objectID;       // 对象ID
    char objectName[10];     // 对象名称
    float lat;               // 纬度
    float lon;               // 经度
    bool useXY;              // 是否使用XY坐标（而非经纬度）
    float x;                 // X坐标
    float y;                 // Y坐标
    bool useOrientation;     // 是否使用姿态角
    float pitch;             // 俯仰角（度）
    float roll;              // 滚转角（度）
    float yaw;               // 偏航角（度）
    bool useHeading;         // 是否使用航向角
    float heading;           // 航向角（度）
    float altitude;          // 海拔高度（米）
    char color[10];          // 显示颜色
    bool useVals;            // 是否使用额外属性值
};

#endif // ACMI_H
