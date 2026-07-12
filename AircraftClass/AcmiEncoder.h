#pragma once

#include "Tacview.h"
#include <string>

namespace tacview {

// 无状态 ACMI 2.2 编码器，便于单独测试和复用。
class AcmiEncoder {
public:
    static std::string header(const Options& options);
    static std::string frameTime(double time);
    static std::string objectDefinition(const ObjectInfo& info,
                                        const ObjectState* state = nullptr);
    static std::string objectUpdate(const ObjectState& state);
    static std::string objectRemoval(ObjectId id);
    static std::string objectId(ObjectId id);
};

} // namespace tacview
