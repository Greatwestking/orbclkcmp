#pragma once

#include <istream>
#include <string>

#include "types.hpp"

namespace orbclkcmp {

void ReadClkHead(std::istream& input);
int ReadClkEpoch(std::istream& input, ClockData& data, std::string* pendingLine = nullptr);
int ReadClkData(
    std::istream& input,
    ClockData& data,
    int epochCount,
    std::string* pendingLine = nullptr);

void ReadClkHeadFile(const std::string& path);
int ReadClkDataFile(const std::string& path, ClockData& data, int epochCount);

}  // namespace orbclkcmp
