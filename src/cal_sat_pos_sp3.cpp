#include "cal_sat_pos_sp3.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "constants.hpp"
#include "lagrange.hpp"

namespace orbclkcmp {
namespace {

SatState Invalid_SP3_State() {
    SatState state;
    state.position = {9999.0, 9999.0, 9999.0};
    state.velocity = {9999.0, 9999.0, 9999.0};
    state.clock = 9999.0;
    state.relativity = 0.0;
    return state;
}

bool All_Coordinates_Invalid(const Eph& eph) {
    for (int component = 0; component < 3; ++component) {
        for (int slot = 0; slot < constants::Sp3EpochSlots; ++slot) {
            if (std::abs(eph.coor[component][slot] - 9999.0) >= 0.1) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

SatState Cal_Sat_Pos_sp3(int gpsWeek, double gpsSec, int PRNIndex,
                               const Sp3Data& sp3Data,
                               const CalSatPosSP3Options& options) {
    const Eph& eph = sp3Data.eph[static_cast<std::size_t>(PRNIndex)];
    if (All_Coordinates_Invalid(eph)) {
        return Invalid_SP3_State();
    }

    std::vector<double> tt(constants::Sp3EpochSlots);
    std::vector<Vector3> coordinates(constants::Sp3EpochSlots);
    for (int i = 0; i < constants::Sp3EpochSlots; ++i) {
        tt[static_cast<std::size_t>(i)] =
            (sp3Data.gpsWeek[i] - gpsWeek) * 604800.0 + sp3Data.gpsSec[i] - gpsSec;
        coordinates[static_cast<std::size_t>(i)] = {
            eph.coor[0][i],
            eph.coor[1][i],
            eph.coor[2][i],
        };
    }

    SatState state;
    const Vector3 interpolatedKm = Lagrange(tt, coordinates, 0.0);
    state.position = {
        interpolatedKm[0] * 1000.0,
        interpolatedKm[1] * 1000.0,
        interpolatedKm[2] * 1000.0,
    };

    const double t1 = 0.1;

    state.relativity = 0.0;
    if (options.relativity) {
        const Vector3 velocityKm = Lagrange_Vel(tt, coordinates, 0.0);
        // Keep ECEF velocity; orbit evaluation and SSR use separate direction rules.
        const Vector3 velocityMeters{velocityKm[0] * 1000.0, velocityKm[1] * 1000.0, velocityKm[2] * 1000.0};
        state.relativity = -2.0 / constants::SpeedOfLight
            * (state.position[0] * velocityMeters[0]
                + state.position[1] * velocityMeters[1]
                + state.position[2] * velocityMeters[2]);
        state.velocity = velocityMeters;
    }

    for (int count = 0; count < 2; ++count) {
        const auto minIt = std::min_element(tt.begin(), tt.end(), [](double a, double b) {
            return std::abs(a) < std::abs(b);
        });
        const auto index = static_cast<std::size_t>(std::distance(tt.begin(), minIt));
        if (std::abs(eph.clk[index] - 999999.999999) <= 1.0) {
            state.clock = 9999.0;
            return state;
        }
        if (count == 0) {
            const double clk0 = eph.clk[index];
            const double t0 = tt[index];
            tt[index] = 999999.0;
            const auto secondMinIt = std::min_element(tt.begin(), tt.end(), [](double a, double b) {
                return std::abs(a) < std::abs(b);
            });
            const auto secondIndex = static_cast<std::size_t>(std::distance(tt.begin(), secondMinIt));
            if (std::abs(eph.clk[secondIndex] - 999999.999999) <= 1.0) {
                state.clock = 9999.0;
                return state;
            }
            const double clk1 = eph.clk[secondIndex];
            const double t2 = tt[secondIndex];
            state.clock = (clk0 + (-t1 - t0) / (t2 - t0) * (clk1 - clk0)) * 1.0e-6;
            return state;
        }
    }

    state.clock = 9999.0;
    return state;
}

}  // namespace orbclkcmp
