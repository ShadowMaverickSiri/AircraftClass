#include "AcmiEncoder.h"

#include <iostream>
#include <string>

int main() {
    tacview::Options options;
    options.dataSource = "Test,Source";
    const std::string header = tacview::AcmiEncoder::header(options);
    if (header.find("FileType=text/acmi/tacview\n") == std::string::npos ||
        header.find("DataSource=Test Source\n") == std::string::npos) {
        std::cerr << "Header encoding failed\n";
        return 1;
    }

    tacview::ObjectInfo info{0x3E9, "Demo,Aircraft", "Air+FixedWing", "Blue"};
    tacview::ObjectState state;
    state.id = info.id;
    state.time = 1.25;
    state.longitude = 116.1234567;
    state.latitude = 40.7654321;
    state.altitude = 5000.0;
    state.roll = 10.0;
    state.pitch = -2.0;
    state.yaw = 90.0;

    const std::string definition = tacview::AcmiEncoder::objectDefinition(info, &state);
    if (definition.find("3E9,Type=Air+FixedWing,Name=Demo Aircraft,Color=Blue,T=") != 0 ||
        tacview::AcmiEncoder::objectRemoval(info.id) != "-3E9\n" ||
        tacview::AcmiEncoder::frameTime(state.time) != "#1.250\n") {
        std::cerr << "Object encoding failed\n";
        return 1;
    }
    return 0;
}
