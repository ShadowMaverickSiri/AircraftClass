#include "ACMI.h"

// ============================================================================
// ACMI 格式字符串模板
// ============================================================================

// 文件头格式模板
// 格式说明：
//   FileType=%s\n              - 文件类型
//   FileVersion=%s\n            - 文件版本
//   0,ReferenceTime=%d-%d-%dT%d:%d:%sZ\n  - 参考时间（ISO 8601格式）
//   0,DataSource=%s\n          - 数据源
//   0,DataRecorder=%s\n         - 数据记录器
//   0,Author=%s\n               - 作者
//   0,Title=%s\n                - 标题
//   0,Comments=%s\n            - 注释
//   0,ReferenceLongitude=%s\n  - 参考经度
//   0,ReferenceLatitude=%s\n    - 参考纬度
const char acmi::headerTemplate[] =
    "FileType=%s\n"
    "FileVersion=%s\n"
    "0,ReferenceTime=%d-%d-%dT%d:%d:%sZ\n"
    "0,DataSource=%s\n"
    "0,DataRecorder=%s\n"
    "0,Author=%s\n"
    "0,Title=%s\n"
    "0,Comments=%s\n"
    "0,ReferenceLongitude=%s\n"
    "0,ReferenceLatitude=%s\n";

// 帧数据格式模板
// 格式说明：
//   #%s\n          - 时间帧标记（例如：#0.1）
//   %s,T=%s|%s|%s|%s|%s|%s|%s|%s|%s\n  - 对象状态更新
//     字段顺序：对象ID, 经度|纬度|高度|滚转|俯仰|偏航|（额外字段）
//   ,              - 空条目标记（用于分隔）
const char acmi::frameTemplate[] = "#%s\n%s,T=%s|%s|%s|%s|%s|%s|%s|%s|%s\n,";

// ============================================================================
// ACMI 类实现
// ============================================================================

// 初始化所有内部状态，重置为默认值
void acmi::begin()
{
    // 清空所有输出缓冲区
    memset(header, '\0', sizeof(header));
    memset(frame, '\0', sizeof(frame));
    memset(extraVals, '\0', sizeof(extraVals));

    // 清空所有元数据字符串
    memset(fileType, '\0', sizeof(fileType));
    memset(fileVersion, '\0', sizeof(fileVersion));
    memset(referenceTime, '\0', sizeof(referenceTime));
    memset(dataSource, '\0', sizeof(dataSource));
    memset(dataRecorder, '\0', sizeof(dataRecorder));
    memset(author, '\0', sizeof(author));
    memset(title, '\0', sizeof(title));
    memset(comments, '\0', sizeof(comments));
    memset(latStr, '\0', sizeof(latStr));
    memset(lonStr, '\0', sizeof(lonStr));

    // 重置时间相关变量
    year = 0;
    month = 0;
    day = 0;
    hour = 0;
    min = 0;
    second = 0;
    refLat = 0;
    refLon = 0;

    // 重置对象状态变量
    timeDelta = 0;
    objectID = 0;
    memset(objectName, '\0', sizeof(objectName));
    lat = 0;
    lon = 0;
    useXY = false;          // 默认使用经纬度而非XY坐标
    x = 0;
    y = 0;
    useOrientation = false; // 默认不使用姿态角
    pitch = 0;
    roll = 0;
    yaw = 0;
    useHeading = false;     // 默认不使用航向角
    heading = 0;
    altitude = 0;
    memset(color, '\0', sizeof(color));
    useVals = false;        // 默认不使用额外属性
}

// 设置文件类型
// 参数：_fileType - 文件类型字符串，标准值为 "text/acmi/tacview"
void acmi::setFileType(const char _fileType[])
{
    // 安全复制：检查缓冲区大小，防止溢出
    if (sizeof(fileType) >= strlen(_fileType) + 1)
        memcpy(fileType, _fileType, strlen(_fileType) + 1);
    else
        memcpy(fileType, _fileType, sizeof(fileType));
}

// 设置文件版本
// 参数：_fileVersion - 文件版本字符串，标准值为 "2.2"
void acmi::setFileVersion(const char _fileVersion[])
{
    if (sizeof(fileVersion) >= strlen(_fileVersion) + 1)
        memcpy(fileVersion, _fileVersion, strlen(_fileVersion) + 1);
    else
        memcpy(fileVersion, _fileVersion, sizeof(fileVersion));
}

// 设置参考时间
// 参数：年、月、日、时、分、秒
void acmi::setReferenceTime(const int& _year, const int& _month, const int& _day,
                             const int& _hour, const int& _min, const float& _second)
{
    year = _year;
    month = _month;
    day = _day;
    hour = _hour;
    min = _min;
    second = _second;
}

// 设置数据源标识
void acmi::setDataSource(const char _dataSource[])
{
    if (sizeof(dataSource) >= strlen(_dataSource) + 1)
        memcpy(dataSource, _dataSource, strlen(_dataSource) + 1);
    else
        memcpy(dataSource, _dataSource, sizeof(dataSource));
}

// 设置数据记录器标识
void acmi::setDataRecorder(const char _dataRecorder[])
{
    if (sizeof(dataRecorder) >= strlen(_dataRecorder) + 1)
        memcpy(dataRecorder, _dataRecorder, strlen(_dataRecorder) + 1);
    else
        memcpy(dataRecorder, _dataRecorder, sizeof(dataRecorder));
}

// 设置作者信息
void acmi::setAuthor(const char _author[])
{
    if (sizeof(author) >= strlen(_author) + 1)
        memcpy(author, _author, strlen(_author) + 1);
    else
        memcpy(author, _author, sizeof(author));
}

// 设置标题
void acmi::setTitle(const char _title[])
{
    if (sizeof(title) >= strlen(_title) + 1)
        memcpy(title, _title, strlen(_title) + 1);
    else
        memcpy(title, _title, sizeof(title));
}

// 设置注释信息
void acmi::setComments(const char _comments[])
{
    if (sizeof(comments) >= strlen(_comments) + 1)
        memcpy(comments, _comments, strlen(_comments) + 1);
    else
        memcpy(comments, _comments, sizeof(comments));
}

// 设置参考经度
// 注意：参数名为 _lat 但实际设置的是经度（原代码命名问题）
void acmi::setReferenceLongitude(const float& _lat)
{
    refLat = _lat;  // 这里实际存储的是经度
}

// 设置参考纬度
// 注意：参数名为 _lon 但实际设置的是纬度（原代码命名问题）
void acmi::setReferenceLatitude(const float& _lon)
{
    refLon = _lon;  // 这里实际存储的是纬度
}

// 生成文件头内容
// 调用此函数前需要先设置所有元数据（setFileType, setFileVersion 等）
// 生成的内容存储在 header 成员变量中
void acmi::createHeader()
{
    // 首先调用 begin() 重置所有状态
    begin();

    // 将秒数转换为字符串（10位精度）
    char secBuff[15];
    snprintf(secBuff, sizeof(secBuff), "%.10f", second);

    // 将经纬度转换为字符串（10位精度）
    char latBuff[15];
    snprintf(latBuff, sizeof(latBuff), "%.10f", refLat);
    char lonBuff[15];
    snprintf(lonBuff, sizeof(lonBuff), "%.10f", refLon);

    // 使用 sprintf_s 格式化文件头
    // 参数顺序对应 headerTemplate 中的占位符
    sprintf_s(header, sizeof(header), headerTemplate,
              fileType, fileVersion,
              year, month, day, hour, min, secBuff,
              dataSource, dataRecorder,
              author, title, comments,
              latBuff, lonBuff);
}

// 设置时间增量（相对上一帧的时间间隔，单位：秒）
void acmi::setTimeDelta(const float& _timeDelta)
{
    timeDelta = _timeDelta;
}

// 设置对象ID（十六进制数）
void acmi::setObjectID(const uint64_t& _objectID)
{
    objectID = _objectID;
}

// 设置对象名称
void acmi::setObjectName(const char _objectName[])
{
    if (sizeof(objectName) >= strlen(_objectName) + 1)
        memcpy(objectName, _objectName, strlen(_objectName) + 1);
    else
        memcpy(objectName, _objectName, sizeof(objectName));
}

// 设置对象位置（经纬度）
// 参数：_lat - 纬度, _lon - 经度
void acmi::setPosition(const float& _lat, const float& _lon)
{
    lat = _lat;
    lon = _lon;
}

// 设置对象位置（XY坐标）
// 此方法会设置 useXY 标志，表示使用XY坐标而非经纬度
void acmi::setPositionXY(const float& _x, const float& _y)
{
    useXY = true;
    x = _x;
    y = _y;
}

// 设置对象姿态角
// 参数：_pitch - 俯仰角（度）, _roll - 滚转角（度）, _yaw - 偏航角（度）
void acmi::setOrientation(const float& _pitch, const float& _roll, const float& _yaw)
{
    useOrientation = true;
    pitch = _pitch;
    roll = _roll;
    yaw = _yaw;
}

// 设置航向角
void acmi::setHeading(const float& _heading)
{
    useHeading = true;
    heading = _heading;
}

// 设置海拔高度（单位：米）
void acmi::setAltitude(const float& _altitude)
{
    altitude = _altitude;
}

// 设置显示颜色
// 常用值：Red, Blue, Green, Orange, Yellow, etc.
void acmi::setColor(const char _color[])
{
    if (sizeof(color) >= strlen(_color) + 1)
        memcpy(color, _color, strlen(_color) + 1);
    else
        memcpy(color, _color, sizeof(color));
}

// 设置自定义属性值
// 参数：_valueName - 属性名称, _value - 属性值
// 此方法将属性添加到 extraVals 中，格式为：,属性名=属性值
// 例如：setValue("Pilot", "John Doe") 会生成 ",Pilot=John Doe"
void acmi::setValue(const char _valueName[], const char _value[])
{
    // 计算当前 extraVals 的末尾位置
    int nextIndex = strlen(extraVals);
    int nameLen = strlen(_valueName) + 1;  // +1 用于 null 终止符
    int valueLen = strlen(_value) + 1;

    // 检查是否有足够的缓冲区空间
    // 需要空间：逗号(1) + 名称 + 等号(1) + 值 + null(1) = nameLen + valueLen + 3
    if ((nextIndex + nameLen + valueLen + 3) <= (int)sizeof(extraVals)) {
        useVals = true;

        // 添加逗号分隔符
        extraVals[nextIndex] = ',';
        nextIndex++;

        // 添加属性名称
        memcpy(extraVals + nextIndex, _valueName, nameLen);
        nextIndex += nameLen;

        // 添加等号
        extraVals[nextIndex] = '=';
        nextIndex++;

        // 添加属性值
        memcpy(extraVals + nextIndex, _value, valueLen);
    }
}

// 创建对象条目（生成帧数据）
// 此函数在原代码中未完成实现
void acmi::createEntry(char _header[])
{
    // 获取当前帧数据的长度
    int nextIndex = strlen(frame);
    // 原代码未完成，此处没有实际功能
}
