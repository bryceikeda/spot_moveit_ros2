#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdio>

namespace BT
{

template <>
inline std::vector<double> convertFromString<std::vector<double>>(StringView str)
{
    std::printf("Converting string to vector<double>: \"%s\"\n", str.data());

    auto parts = BT::splitString(str, ';'); 

    if (parts.empty())
    {
        throw BT::RuntimeError(
            "Invalid input for std::vector<double>. Expected semicolon-separated list.");
    }

    std::vector<double> out;
    out.reserve(parts.size());

    for (const auto& p : parts)
    {
        out.push_back(std::stod(std::string(p.data(), p.size())));
    }

    return out;
}


}  // namespace BT
