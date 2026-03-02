#ifndef ACMI_H
#define ACMI_H

#include <cstring>
#include <cstdint>
#include <cstdio>

const uint64_t MAX_NUM_OBJS = 0xFFFFFFFFFFFFFFFE;

// ============================================================================
// ACMI 格式生成器
// 用于生成 Tacview 通用飞行数据交换格式
// ============================================================================
class acmi {
public:
    char header[1000];
    char frame[1000];
    char extraVals[1000];

    void begin();

    // 文件元数据设置
    void setFileType(const char _fileType[]);
    void setFileVersion(const char _fileVersion[]);
    void setReferenceTime(const int& _year = 0, const int& _month = 0, const int& _day = 0,
                           const int& _hour = 0, const int& _min = 0, const float& _second = 0);
    void setDataSource(const char _dataSource[]);
    void setDataRecorder(const char _dataRecorder[]);
    void setAuthor(const char _author[]);
    void setTitle(const char _title[]);
    void setComments(const char _comments[]);
    void setReferenceLongitude(const float& _lat = 0);
    void setReferenceLatitude(const float& _lon = 0);

    void createHeader();

    // 对象属性设置
    void setTimeDelta(const float& _timeDelta);
    void setObjectID(const uint64_t& _objectID);
    void setObjectName(const char _objectName[]);
    void setPosition(const float& _lat, const float& _lon);
    void setPositionXY(const float& _x, const float& _y);
    void setOrientation(const float& _pitch, const float& _roll, const float& _yaw);
    void setHeading(const float& _heading);
    void setAltitude(const float& _altitude = 0);
    void setColor(const char _color[]);
    void setValue(const char _valueName[], const char _value[]);

    void createEntry(char _header[]);

private:
    static const char headerTemplate[];
    static const char frameTemplate[];

    char fileType[40];
    char fileVersion[10];
    char referenceTime[40];
    char dataSource[40];
    char dataRecorder[40];
    char author[40];
    char title[40];
    char comments[40];
    char latStr[40];
    char lonStr[40];

    int year;
    int month;
    int day;
    int hour;
    int min;
    float second;
    float refLat;
    float refLon;

    float timeDelta;
    uint64_t objectID;
    char objectName[10];
    float lat;
    float lon;
    bool useXY;
    float x;
    float y;
    bool useOrientation;
    float pitch;
    float roll;
    float yaw;
    bool useHeading;
    float heading;
    float altitude;
    char color[10];
    bool useVals;
};

#endif // ACMI_H
