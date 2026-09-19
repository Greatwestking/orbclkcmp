#pragma once

#include <array>
#include <vector>

#include "types.hpp"

namespace orbclkcmp {

struct CalSatPosOptions {
    RuntimeOptions runtime;
    std::array<double, 6> dcb{};
    Vector3 tgdBias{};
    BDSFreq bdsfreq = BDSFreq::B1B3;
    bool bdsBrdClockB3 = true;
    bool useBroadcastTGD = false;
    bool useSSRNavIode = false;
    bool bdsSSRIsB2b = false;
    double ssrIode = -2.0;
};

// Calculate satellite state from broadcast ephemeris.
SatState Cal_Sat_Pos_nav(char system, int gpsWeek, double gpsSec, int prn,
                         const std::vector<NavRecord>& records,
                         const CalSatPosOptions& options = {});

// Calculate GLONASS satellite state from broadcast ephemeris.
SatState Cal_Sat_Pos_g(int gpsWeek, double gpsSec, int prn,
                          const std::vector<GlonassNavRecord>& records,
                          const CalSatPosOptions& options = {});

}  // namespace orbclkcmp
