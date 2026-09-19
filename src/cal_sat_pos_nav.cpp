#include "cal_sat_pos_nav.hpp"

#include <cmath>
#include <cstddef>

#include "constants.hpp"

namespace orbclkcmp {
namespace {

constexpr double RelativityF = -4.442807633e-10;
constexpr double GalileoRelativityF = -4.442807309e-10;

SatState Invalid_State() {
    SatState state;
    state.position = {9999.0, 9999.0, 9999.0};
    state.velocity = {9999.0, 9999.0, 9999.0};
    state.clock = 9999.0;
    state.relativity = 0.0;
    return state;
}

double Total_Seconds(int week, double seconds) {
    return week * 604800.0 + seconds;
}

bool Has_DCB(double value) {
    return std::abs(value - 99.0) > 1.0e-12;
}

double BDS_Brd_Clock_Offset(const CalSatPosOptions& options, const NavRecord& record, bool& valid) {
    // Convert the B3-based broadcast clock to the requested frequency combination.
    if (options.bdsBrdClockB3) {
        valid = true;
        return 0.0;
    }
    if (options.bdsfreq == BDSFreq::B3) {
        valid = true;
        return 0.0;
    }
    if (options.useBroadcastTGD) {
        valid = true;
        if (options.bdsfreq == BDSFreq::B1B2) {
            if (options.runtime.IsCNAV) {
                return 2.2606 * record.tgd[0] - 1.2606 * record.tgd[1];
            }
            return 2.4872 * record.tgd[0] - 1.4872 * record.tgd[1];
        }
        return 2.9437 * record.tgd[0];
    }
    if (options.bdsfreq == BDSFreq::B1B2) {
        valid = Has_DCB(options.dcb[3]) && Has_DCB(options.dcb[4]);
        return valid ? 2.2606 * options.dcb[3] - 1.2606 * options.dcb[4] : 0.0;
    }
    valid = Has_DCB(options.dcb[0]);
    return valid ? 2.9437 * options.dcb[0] : 0.0;
}

bool Usable_For_System(char system, const NavRecord& record, const CalSatPosOptions& options) {
    if (system == 'G') {
        return record.health == 0.0 || options.runtime.IsCNAV;
    }
    if (system == 'J') {
        return record.health <= 64.0 || options.runtime.IsCNAV;
    }
    if (system == 'C') {
        return record.health == 0.0 && (record.iode < 20.0 || options.runtime.IsCNAV);
    }
    if (system == 'E') {
        return record.health == 0.0 && record.code > 512.0;
    }
    return false;
}

double BDS_SSR_IOD(const NavRecord& record, const CalSatPosOptions& options) {
    double time = std::fmod(record.gpsSec, 604800.0);
    if (time < 0.0) {
        time += 604800.0;
    }
    if (options.bdsSSRIsB2b) {
        return static_cast<double>(static_cast<int>(time / 3600.0));
    }
    double iod = std::fmod(time / 720.0, 240.0);
    if (iod < 0.0) {
        iod += 240.0;
    }
    return iod;
}

bool Usable_For_SSR_IODE(char system, const NavRecord& record, const CalSatPosOptions& options) {
    if (!options.useSSRNavIode || options.ssrIode < 0.0) {
        return false;
    }
    if (system == 'C') {
        return std::abs(BDS_SSR_IOD(record, options) - options.ssrIode) <= 1.0e-9;
    }
    if (std::abs(record.iode - options.ssrIode) > 1.0e-9) {
        return false;
    }
    if (!Usable_For_System(system, record, options)) {
        return false;
    }
    if (system == 'E') {
        return record.code > 512.0;
    }
    return true;
}

double Max_Age_For_System(char system) {
    if (system == 'C') {
        return 3660.0;
    }
    if (system == 'E') {
        return 7260.0;
    }
    return 9000.0;
}

double Max_Age_For_Selected_Record(char system, const CalSatPosOptions& options) {
    (void)options;
    return Max_Age_For_System(system);
}

double Scan_Exit_Threshold_For_System(char system) {
    if (system == 'C') {
        return 7200.0;
    }
    if (system == 'G' || system == 'J') {
        return 9000.0;
    }
    return 0.0;
}

const NavRecord* selectRecord(char system, int gpsWeek, double gpsSec, int prn,
                              const std::vector<NavRecord>& records,
                              const CalSatPosOptions& options,
                              double& tk) {
    const NavRecord* best = nullptr;
    // Start with the default time offset.
    double bestDt = Total_Seconds(gpsWeek, gpsSec);
    const double scanExitThreshold = Scan_Exit_Threshold_For_System(system);

    for (const NavRecord& record : records) {
        const double dt = Total_Seconds(gpsWeek, gpsSec) - Total_Seconds(record.gpsWeek, record.gpsSec);
        const bool match = options.useSSRNavIode
            ? Usable_For_SSR_IODE(system, record, options)
            : Usable_For_System(system, record, options);
        if (match && std::abs(dt) < std::abs(bestDt)) {
            best = &record;
            bestDt = dt;
            continue;
        }
        if (scanExitThreshold > 0.0 && std::abs(dt) - std::abs(bestDt) > scanExitThreshold) {
            break;
        }
    }

    tk = bestDt;
    return best;
}

void Apply_BDS_GEO_Rotation(int prn, double earthRotation, double tk, SatState& state) {
    if (!(prn < 6 || prn == 17 || prn == 18)) {
        return;
    }

    const double cos5 = std::cos(5.0 * constants::Pi / 180.0);
    const double sin5 = std::sin(5.0 * constants::Pi / 180.0);
    const double cosWt = std::cos(earthRotation * tk);
    const double sinWt = std::sin(earthRotation * tk);

    const double x = state.position[0];
    const double y = state.position[1];
    const double z = state.position[2];
    state.position[0] = x * cosWt + y * cos5 * sinWt - z * sin5 * sinWt;
    state.position[1] = -x * sinWt + y * cos5 * cosWt - z * sin5 * cosWt;
    state.position[2] = y * sin5 + z * cos5;

    const double vx = state.velocity[0];
    const double vy = state.velocity[1];
    const double vz = state.velocity[2];
    state.velocity[0] = vx * cosWt + vy * cos5 * sinWt - vz * sin5 * sinWt
        - x * sinWt * earthRotation + y * cos5 * cosWt * earthRotation
        - z * sin5 * cosWt * earthRotation;
    state.velocity[1] = -vx * sinWt + vy * cos5 * cosWt - vz * sin5 * cosWt
        - x * cosWt * earthRotation - y * cos5 * sinWt * earthRotation
        + z * sin5 * sinWt * earthRotation;
    state.velocity[2] = vy * sin5 + vz * cos5;
}

Vector3 GLONASS_Acceleration(double x, double y, double z, double vx, double vy,
                            double ax0, double ay0, double az0) {
    const double r = std::sqrt(x * x + y * y + z * z);
    const double a1 = constants::GlonassEarthGravityKm / (r * r * r);
    const double a2 = 1.5 * constants::GlonassJ2 * constants::GlonassEarthGravityKm
        * constants::GlonassEarthSemiMajorAxisKm * constants::GlonassEarthSemiMajorAxisKm
        / std::pow(r, 5);
    const double omega = constants::GlonassEarthAngularVelocity;

    return {
        -a1 * x + a2 * x * (1.0 - 5.0 * z * z / (r * r)) + omega * omega * x + 2.0 * omega * vy + ax0,
        -a1 * y + a2 * y * (1.0 - 5.0 * z * z / (r * r)) + omega * omega * y - 2.0 * omega * vx + ay0,
        -a1 * z + a2 * z * (3.0 - 5.0 * z * z / (r * r)) + az0,
    };
}

double Glonass_SSR_IODE(const GlonassNavRecord& record) {
    double value = std::fmod(record.gpsSec + 10800.0, 86400.0);
    if (value < 0.0) {
        value += 86400.0;
    }
    return value / 900.0;
}

const GlonassNavRecord* selectGlonassRecord(int gpsWeek, double gpsSec,
                                            const std::vector<GlonassNavRecord>& records,
                                            const CalSatPosOptions& options,
                                            double& tk) {
    const GlonassNavRecord* best = nullptr;
    double bestDt = Total_Seconds(gpsWeek, gpsSec);

    for (const GlonassNavRecord& record : records) {
        const double dt = Total_Seconds(gpsWeek, gpsSec) - Total_Seconds(record.gpsWeek, record.gpsSec);
        bool match = std::abs(dt) < std::abs(bestDt) + 0.2 && record.health == 0.0;
        if (options.useSSRNavIode) {
            const double iode = Glonass_SSR_IODE(record);
            match = ((options.ssrIode < -1.0 && std::abs(dt) < std::abs(bestDt) + 0.2)
                || std::abs(iode - options.ssrIode) < 0.001)
                && record.health == 0.0;
        }
        if (!match) {
            continue;
        }
        best = &record;
        bestDt = dt;
    }

    tk = bestDt;
    return best;
}

}  // namespace

SatState Cal_Sat_Pos_nav(char system, int gpsWeek, double gpsSec, int prn,
                         const std::vector<NavRecord>& records,
                         const CalSatPosOptions& options) {
    double tk = 0.0;
    const NavRecord* selected = selectRecord(system, gpsWeek, gpsSec, prn, records, options, tk);
    if (selected == nullptr || std::abs(tk) > Max_Age_For_Selected_Record(system, options) || selected->a0 == 0.0) {
        return Invalid_State();
    }
    if (system == 'C' && (selected->crs == 0.0 || selected->sqrtA == 0.0)) {
        return Invalid_State();
    }

    double earthRotation = constants::Wgs84EarthAngularVelocity;
    double gravity = constants::Wgs84EarthGravity;
    double relativityF = RelativityF;
    if (system == 'C') {
        earthRotation = constants::Cgcs2000EarthAngularVelocity;
        gravity = constants::Cgcs2000EarthGravity;
    } else if (system == 'E') {
        earthRotation = constants::GalileoEarthAngularVelocity;
        gravity = constants::GalileoEarthGravity;
        relativityF = GalileoRelativityF;
    }

    const double e = selected->e;
    double semiMajorAxis = selected->sqrtA * selected->sqrtA;
    const double n0 = std::sqrt(gravity / (semiMajorAxis * semiMajorAxis * semiMajorAxis));
    double n = n0 + selected->deltaN;
    if (options.runtime.IsCNAV) {
        n = n0 + selected->deltaN + 0.5 * selected->nDot * tk;
        semiMajorAxis = selected->sqrtA * selected->sqrtA + selected->aDot * tk;
    }

    const double meanAnomaly = selected->m0 + n * tk;
    double eccentricAnomaly0 = meanAnomaly;
    double eccentricAnomaly = meanAnomaly;
    do {
        eccentricAnomaly = meanAnomaly + e * std::sin(eccentricAnomaly0);
        if (std::abs(eccentricAnomaly - eccentricAnomaly0) < 1e-12) {
            break;
        }
        eccentricAnomaly0 = eccentricAnomaly;
    } while (true);

    const double eccentricAnomalyDot = n / (1.0 - e * std::cos(eccentricAnomaly));
    double trueAnomaly = 2.0 * std::atan(std::tan(eccentricAnomaly / 2.0)
        * std::sqrt((1.0 + e) / (1.0 - e)));
    if (trueAnomaly < 0.0) {
        trueAnomaly += 2.0 * constants::Pi;
    }
    const double phi = trueAnomaly + selected->omega;
    const double phiDot = std::sqrt(1.0 + e) / std::sqrt(1.0 - e)
        * std::pow(std::cos(trueAnomaly / 2.0), 2)
        / std::pow(std::cos(eccentricAnomaly / 2.0), 2)
        * eccentricAnomalyDot;

    const double du = selected->cus * std::sin(2.0 * phi) + selected->cuc * std::cos(2.0 * phi);
    const double dr = selected->crs * std::sin(2.0 * phi) + selected->crc * std::cos(2.0 * phi);
    const double di = selected->cis * std::sin(2.0 * phi) + selected->cic * std::cos(2.0 * phi);

    const double u = phi + du;
    const double uDot = (1.0 + 2.0 * selected->cus * std::cos(2.0 * phi)
        - 2.0 * selected->cuc * std::sin(2.0 * phi)) * phiDot;
    const double radius = semiMajorAxis * (1.0 - e * std::cos(eccentricAnomaly)) + dr;
    const double radiusDot = eccentricAnomalyDot * semiMajorAxis * e * std::sin(eccentricAnomaly)
        + 2.0 * (selected->crs * std::cos(2.0 * phi) - selected->crc * std::sin(2.0 * phi)) * phiDot;
    const double inclination = selected->i0 + di + selected->idot * tk;
    const double inclinationDot = 2.0 * (selected->cis * std::cos(2.0 * phi)
        - selected->cic * std::sin(2.0 * phi)) * phiDot + selected->idot;

    const double xOrbital = radius * std::cos(u);
    const double yOrbital = radius * std::sin(u);
    const double xOrbitalDot = radiusDot * std::cos(u) - radius * std::sin(u) * uDot;
    const double yOrbitalDot = radiusDot * std::sin(u) + radius * std::cos(u) * uDot;

    double omega = selected->omega0 + (selected->omegaDot - earthRotation) * tk - earthRotation * selected->toe;
    double omegaDot = selected->omegaDot - earthRotation;
    if (system == 'C' && (prn < 6 || prn == 17)) {
        omega = selected->omega0 + selected->omegaDot * tk - earthRotation * selected->toe;
        omegaDot = selected->omegaDot;
    }

    SatState state;
    state.position[0] = xOrbital * std::cos(omega) - yOrbital * std::cos(inclination) * std::sin(omega);
    state.position[1] = xOrbital * std::sin(omega) + yOrbital * std::cos(inclination) * std::cos(omega);
    state.position[2] = yOrbital * std::sin(inclination);
    state.velocity[0] = xOrbitalDot * std::cos(omega)
        - yOrbitalDot * std::sin(omega) * std::cos(inclination)
        + yOrbital * std::sin(omega) * std::sin(inclination) * inclinationDot
        - (xOrbital * std::sin(omega) + yOrbital * std::cos(omega) * std::cos(inclination)) * omegaDot;
    state.velocity[1] = xOrbitalDot * std::sin(omega)
        + yOrbitalDot * std::cos(omega) * std::cos(inclination)
        - yOrbital * std::cos(omega) * std::sin(inclination) * inclinationDot
        + (xOrbital * std::cos(omega) - yOrbital * std::sin(omega) * std::cos(inclination)) * omegaDot;
    state.velocity[2] = yOrbitalDot * std::sin(inclination) + yOrbital * std::cos(inclination) * inclinationDot;

    if (system == 'C') {
        Apply_BDS_GEO_Rotation(prn, earthRotation, tk, state);
    }

    // Apply earth-rotation correction to velocity.
    const double earthRotationSin = std::sin(earthRotation);
    state.velocity[0] -= earthRotationSin * state.position[1];
    state.velocity[1] += earthRotationSin * state.position[0];

    state.clock = selected->a0 + tk * selected->a1 + tk * tk * selected->a2;
    if (system == 'C') {
        bool clockOffsetValid = true;
        state.clock -= BDS_Brd_Clock_Offset(options, *selected, clockOffsetValid);
        if (!clockOffsetValid) {
            state.clock = 9999.0;
            return state;
        }
        if (options.bdsfreq == BDSFreq::B1B3) {
            state.clock += options.tgdBias[2] / constants::SpeedOfLight;
        }
    }
    state.relativity = constants::SpeedOfLight * relativityF * e * std::sqrt(semiMajorAxis)
        * std::sin(eccentricAnomaly);
    return state;
}

SatState Cal_Sat_Pos_g(int gpsWeek, double gpsSec, int prn,
                          const std::vector<GlonassNavRecord>& records,
                          const CalSatPosOptions& options) {
    double tk = 0.0;
    const GlonassNavRecord* selected = selectGlonassRecord(gpsWeek, gpsSec, records, options, tk);
    if (selected == nullptr || std::abs(tk) > 3600.1 || selected->x == 0.0
        || selected->y == 0.0 || selected->z == 0.0) {
        return Invalid_State();
    }

    double step = tk < 0.0 ? -30.0 : 30.0;
    const int count = static_cast<int>(std::abs(tk) / std::abs(step));
    double x = selected->x;
    double y = selected->y;
    double z = selected->z;
    double vx = selected->vx;
    double vy = selected->vy;
    double vz = selected->vz;
    const double ax0 = selected->ax;
    const double ay0 = selected->ay;
    const double az0 = selected->az;
    double outputX = x;
    double outputY = y;
    double outputZ = z;
    double outputVx = vx;
    double outputVy = vy;
    double outputVz = vz;

    for (int i = 0; i <= count; ++i) {
        const double remaining = tk - i * step;
        if (std::abs(remaining) < std::abs(step)) {
            step = remaining;
        }

        const auto k1a = GLONASS_Acceleration(x, y, z, vx, vy, ax0, ay0, az0);
        const Vector3 k1v{vx, vy, vz};

        const auto k2a = GLONASS_Acceleration(
            x + k1v[0] * step / 2.0,
            y + k1v[1] * step / 2.0,
            z + k1v[2] * step / 2.0,
            vx + k1a[0] * step / 2.0,
            vy + k1a[1] * step / 2.0,
            ax0, ay0, az0);
        const Vector3 k2v{vx + k1a[0] * step / 2.0, vy + k1a[1] * step / 2.0, vz + k1a[2] * step / 2.0};

        const auto k3a = GLONASS_Acceleration(
            x + k2v[0] * step / 2.0,
            y + k2v[1] * step / 2.0,
            z + k2v[2] * step / 2.0,
            vx + k2a[0] * step / 2.0,
            vy + k2a[1] * step / 2.0,
            ax0, ay0, az0);
        const Vector3 k3v{vx + k2a[0] * step / 2.0, vy + k2a[1] * step / 2.0, vz + k2a[2] * step / 2.0};

        const auto k4a = GLONASS_Acceleration(
            x + k3v[0] * step,
            y + k3v[1] * step,
            z + k3v[2] * step,
            vx + k3a[0] * step,
            vy + k3a[1] * step,
            ax0, ay0, az0);
        const Vector3 k4v{vx + k3a[0] * step, vy + k3a[1] * step, vz + k3a[2] * step};
        outputX = x + k3v[0] * step;
        outputY = y + k3v[1] * step;
        outputZ = z + k3v[2] * step;
        outputVx = k4v[0];
        outputVy = k4v[1];
        outputVz = k4v[2];

        vx += (k1a[0] + 2.0 * k2a[0] + 2.0 * k3a[0] + k4a[0]) * step / 6.0;
        vy += (k1a[1] + 2.0 * k2a[1] + 2.0 * k3a[1] + k4a[1]) * step / 6.0;
        vz += (k1a[2] + 2.0 * k2a[2] + 2.0 * k3a[2] + k4a[2]) * step / 6.0;
        x += (k1v[0] + 2.0 * k2v[0] + 2.0 * k3v[0] + k4v[0]) * step / 6.0;
        y += (k1v[1] + 2.0 * k2v[1] + 2.0 * k3v[1] + k4v[1]) * step / 6.0;
        z += (k1v[2] + 2.0 * k2v[2] + 2.0 * k3v[2] + k4v[2]) * step / 6.0;
    }

    SatState state;
    state.position = {outputX * 1000.0 + 0.003, outputY * 1000.0 + 0.001, outputZ * 1000.0 + 0.02};
    // Use zero Rotation for this GLONASS branch.
    state.velocity = {outputVx * 1000.0, outputVy * 1000.0, outputVz * 1000.0};
    state.relativity = -(outputX * outputVx + outputY * outputVy + outputZ * outputVz)
        * 2.0 / constants::SpeedOfLight * 1000.0 * 1000.0;
    state.clock = selected->a0 + tk * selected->a1 - state.relativity / constants::SpeedOfLight;

    const double f1 = (1602.0 + selected->frequency * 0.5625) * 1.0e6;
    const double f2 = (1246.0 + selected->frequency * 0.4375) * 1.0e6;
    const double ionosphereFreeFactor = f2 * f2 / (f1 + f2) / (f1 - f2);
    state.clock += options.dcb[1] - ionosphereFreeFactor * options.dcb[0];
    return state;
}

}  // namespace orbclkcmp
