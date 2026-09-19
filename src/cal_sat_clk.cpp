#include "cal_sat_clk.hpp"

#include <cmath>
#include <cstddef>

namespace orbclkcmp {

double Cal_Sat_Clk(int gpsWeek, double gpsSec, int PRNIndex, const ClockData& clkData) {
    const double dGPST0 = (clkData.gpsWeek[0] - gpsWeek) * 604800.0 + clkData.gpsSec[0] - gpsSec;
    const double dGPST1 = (clkData.gpsWeek[1] - gpsWeek) * 604800.0 + clkData.gpsSec[1] - gpsSec;
    if (std::abs(dGPST0) > 300.0 || std::abs(dGPST1) > 300.0) {
        return 9999.0;
    }

    const auto& record = clkData.as[static_cast<std::size_t>(PRNIndex)];
    const double clk1 = record.clk[0];
    const double clk2 = record.clk[1];
    if (std::abs(clk1 - 9999.0) > 0.01 && std::abs(clk2 - 9999.0) > 0.01) {
        return clk1 - dGPST0 / (dGPST1 - dGPST0) * (clk2 - clk1);
    }
    if (std::abs(clk1 - 9999.0) > 0.01 && std::abs(dGPST0) < 0.01) {
        return clk1;
    }
    if (std::abs(clk2 - 9999.0) > 0.01 && std::abs(dGPST1) < 0.01) {
        return clk2;
    }
    return 9999.0;
}

}  // namespace orbclkcmp
