#pragma once
#include "behaviortree_cpp/bt_factory.h"

struct OrientationXYZW
{

    double x;
    double y;
    double z;
    double w;
};

namespace BT
{

// Expected format:
// "x;y;z;w"
template <>
inline OrientationXYZW convertFromString(BT::StringView str)
{
    printf("Converting string to OrientationXYZW: \"%s\"\n", str.data());

    auto parts = BT::splitString(str, ';');
    if (parts.size() != 4)
    {
        throw BT::RuntimeError(
            "Invalid input for OrientationXYZW. Expected 4 values: x;y;z;w");
    }

    OrientationXYZW output;
    output.x = BT::convertFromString<double>(parts[0]);
    output.y = BT::convertFromString<double>(parts[1]);
    output.z = BT::convertFromString<double>(parts[2]);
    output.w = BT::convertFromString<double>(parts[3]);

    return output;
}

} // namespace BT
