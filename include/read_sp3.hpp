#pragma once

#include <istream>
#include <string>

#include "prn_index.hpp"
#include "types.hpp"

namespace orbclkcmp {

Sp3Head ReadSP3Head(std::istream& input, std::string* firstEpochLine = nullptr);
Sp3Head ReadSP3HeadFile(const std::string& path, std::string* firstEpochLine = nullptr);
int ReadSP3Data(
    std::istream& input,
    const Sp3Head& head,
    Sp3Data& data,
    int epochCount,
    std::string firstEpochLine = {});
int ReadSP3Data(
    std::istream& input,
    const Sp3Head& head,
    Sp3Data& data,
    int epochCount,
    std::string* pendingEpochLine);
int ReadSP3DataFile(const std::string& path, const Sp3Head& head, Sp3Data& data, int epochCount);

}  // namespace orbclkcmp
