#pragma once

#include "types.hpp"

namespace orbclkcmp {

// Get precise clock values for one satellite, including interpolate.
double Cal_Sat_Clk(int gpsWeek, double gpsSec, int PRNIndex, const ClockData& clkData);

}  // namespace orbclkcmp
