#pragma once

#include <array>
#include <string>

#include "constants.hpp"

namespace orbclkcmp {

struct MGEXDCB {
    std::array<std::array<double, 6>, constants::CNum + 1> bds{};
    std::array<std::array<double, 2>, constants::RNum + 1> glonass{};

    MGEXDCB();
};

// Read MGEX BDS/GLONASS DCB values from a Bias-SINEX file.
MGEXDCB Read_MGEXDCB(const std::string& path);

}  // namespace orbclkcmp
