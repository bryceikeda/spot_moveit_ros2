#pragma once
#include "behaviortree_cpp/bt_factory.h"

struct AxisToleranceXYZ
{

    double x;
    double y;
    double z;
};

namespace BT
{

// Expected format:
// "x;y;z"
template <>
inline AxisToleranceXYZ convertFromString(BT::StringView str)
{
    printf("Converting string to AxisToleranceXYZ: \"%s\"\n", str.data());

    auto parts = BT::splitString(str, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError(
            "Invalid input for AxisToleranceXYZ. Expected 3 values: x;y;z");
    }

    AxisToleranceXYZ output;
    output.x = BT::convertFromString<double>(parts[0]);
    output.y = BT::convertFromString<double>(parts[1]);
    output.z = BT::convertFromString<double>(parts[2]);

    return output;
}

} // namespace BT