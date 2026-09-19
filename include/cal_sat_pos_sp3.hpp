#pragma once

#include "types.hpp"

namespace orbclkcmp {

struct CalSatPosSP3Options {
    bool relativity = true;
};

// Calculate SP3 state. With relativity enabled, velocity is the ECEF derivative.
SatState Cal_Sat_Pos_sp3(int gpsWeek, double gpsSec, int PRNIndex,
                               const Sp3Data& sp3Data,
                               const CalSatPosSP3Options& options = {});

}  // namespace orbclkcmp
