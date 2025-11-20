#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdio>

namespace BT
{

// Specialization: convert a comma-separated string into a vector of strings
template <>
inline std::vector<std::string> convertFromString<std::vector<std::string>>(StringView str)
{
    std::printf("Converting string to vector<string>: \"%s\"\n", str.data());

    auto parts = BT::splitString(str, ';'); 

    if (parts.empty())
    {
        throw BT::RuntimeError(
            "Invalid input for std::vector<std::string>. Expected semi-colon-separated list.");
    }

    std::vector<std::string> out;
    out.reserve(parts.size());

    for (const auto& p : parts)
    {
        out.push_back(std::string(p.data(), p.size()));
    }

    return out;
}

}  // namespace BT
