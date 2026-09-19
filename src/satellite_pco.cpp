#include "satellite_pco.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "constants.hpp"
#include "cross.hpp"

namespace orbclkcmp {

namespace {

constexpr double Pi = 3.141592653589793238462643383279502884;
constexpr double DegreesToRadians = Pi / 180.0;
constexpr double ArcsecondsToRadians = DegreesToRadians / 3600.0;
constexpr double GpsEpochMjd = 44244.0;

Vector3 Normalize(const Vector3& value) {
    const double length = std::sqrt(value[0] * value[0]
                                  + value[1] * value[1]
                                  + value[2] * value[2]);
    if (length == 0.0) {
        return {};
    }
    return {value[0] / length, value[1] / length, value[2] / length};
}

Matrix3 Multiply(const Matrix3& left, const Matrix3& right) {
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int index = 0; index < 3; ++index) {
                result[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)]
                    += left[static_cast<std::size_t>(row)][static_cast<std::size_t>(index)]
                     * right[static_cast<std::size_t>(index)][static_cast<std::size_t>(column)];
            }
        }
    }
    return result;
}

Vector3 Multiply(const Matrix3& matrix, const Vector3& value) {
    return {
        matrix[0][0] * value[0] + matrix[0][1] * value[1] + matrix[0][2] * value[2],
        matrix[1][0] * value[0] + matrix[1][1] * value[1] + matrix[1][2] * value[2],
        matrix[2][0] * value[0] + matrix[2][1] * value[1] + matrix[2][2] * value[2],
    };
}

Matrix3 Transpose(const Matrix3& matrix) {
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)]
                = matrix[static_cast<std::size_t>(column)][static_cast<std::size_t>(row)];
        }
    }
    return result;
}

Matrix3 Rotate_Y(double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{cosine, 0.0, -sine},
             {0.0, 1.0, 0.0},
             {sine, 0.0, cosine}}};
}

Matrix3 Rotate_Z(double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{cosine, sine, 0.0},
             {-sine, cosine, 0.0},
             {0.0, 0.0, 1.0}}};
}

double Positive_Remainder(double value, double divisor) {
    const double result = std::fmod(value, divisor);
    return result < 0.0 ? result + divisor : result;
}

Vector3 Sun_Position_ECI(double utcMjd) {
    const double centuries = (utcMjd - 51544.5) / 36525.0;
    const double obliquity = (23.439291 - 0.0130042 * centuries) * DegreesToRadians;
    const double meanAnomaly = (357.5277233 + 35999.05034 * centuries) * DegreesToRadians;
    const double longitude = (280.460 + 36000.770 * centuries
        + 1.914666471 * std::sin(meanAnomaly)
        + 0.019994643 * std::sin(2.0 * meanAnomaly)) * DegreesToRadians;
    const double distance = 149597870691.0
        * (1.000140612 - 0.016708617 * std::cos(meanAnomaly)
           - 0.000139589 * std::cos(2.0 * meanAnomaly));
    return {
        distance * std::cos(longitude),
        distance * std::cos(obliquity) * std::sin(longitude),
        distance * std::sin(obliquity) * std::sin(longitude),
    };
}

Matrix3 ECI_To_ECEF(double gpsMjd, double leapSeconds) {
    const double utcMjd = gpsMjd - leapSeconds / 86400.0;
    const double centuries = (gpsMjd - 51544.5 + (19.0 + 32.184) / 86400.0) / 36525.0;
    const double centuries2 = centuries * centuries;
    const double centuries3 = centuries2 * centuries;

    const double zeta = (2306.2181 * centuries + 0.30188 * centuries2
                         + 0.017998 * centuries3) * ArcsecondsToRadians;
    const double theta = (2004.3109 * centuries - 0.42665 * centuries2
                          - 0.041833 * centuries3) * ArcsecondsToRadians;
    const double z = (2306.2181 * centuries + 1.09468 * centuries2
                      + 0.018203 * centuries3) * ArcsecondsToRadians;
    const Matrix3 precession = Multiply(Multiply(Rotate_Z(-z), Rotate_Y(theta)), Rotate_Z(-zeta));

    const double utcDay = std::trunc(utcMjd);
    const double secondsOfDay = Positive_Remainder(utcMjd, 1.0) * 86400.0;
    const double t1 = (utcDay - 51544.5) / 36525.0;
    const double gmstSeconds = 24110.54841 + 8640184.812866 * t1
        + 0.093104 * t1 * t1 - 6.2e-6 * t1 * t1 * t1
        + 1.002737909350795 * secondsOfDay;
    const double gmst = Positive_Remainder(gmstSeconds, 86400.0) * Pi / 43200.0;
    return Multiply(Rotate_Z(gmst), precession);
}

}  // namespace

Vector3 Satellite_PCO_To_ECEF(int gpsWeek, double gpsSec, int leapSeconds,
                              const Vector3& satellitePosition,
                              const Vector3& pco) {
    const double usedLeapSeconds = leapSeconds > 0 ? static_cast<double>(leapSeconds) : 18.0;
    const double gpsMjd = GpsEpochMjd
        + static_cast<double>(gpsWeek) * 7.0 + gpsSec / 86400.0;
    const Matrix3 crsToTrs = ECI_To_ECEF(gpsMjd, usedLeapSeconds);
    const Matrix3 trsToCrs = Transpose(crsToTrs);
    const Vector3 satelliteCrs = Multiply(trsToCrs, satellitePosition);
    const Vector3 sunCrs = Sun_Position_ECI(gpsMjd - usedLeapSeconds / 86400.0);

    const Vector3 zAxis = Normalize({-satelliteCrs[0], -satelliteCrs[1], -satelliteCrs[2]});
    const Vector3 sunFromSatellite{
        sunCrs[0] - satelliteCrs[0],
        sunCrs[1] - satelliteCrs[1],
        sunCrs[2] - satelliteCrs[2],
    };
    const Vector3 yAxis = Normalize(Cross(
        {-satelliteCrs[0], -satelliteCrs[1], -satelliteCrs[2]}, sunFromSatellite));
    const Vector3 xAxis = Cross(yAxis, zAxis);

    const Vector3 pcoCrs{
        xAxis[0] * pco[0] + yAxis[0] * pco[1] + zAxis[0] * pco[2],
        xAxis[1] * pco[0] + yAxis[1] * pco[1] + zAxis[1] * pco[2],
        xAxis[2] * pco[0] + yAxis[2] * pco[1] + zAxis[2] * pco[2],
    };
    return Multiply(crsToTrs, pcoCrs);
}

Vector3 BDS_PCO_To_ECEF(int gpsWeek, double gpsSec, int leapSeconds,
                        const Vector3& position, const Vector3& velocity,
                        const Vector3& pco, bool isGeo, double* betaDeg) {
    for (const Vector3* value : {&position, &velocity, &pco}) {
        for (double component : *value) {
            if (!std::isfinite(component)) {
                throw std::runtime_error("BDS PCO: non-finite position, velocity or offset");
            }
        }
    }
    const auto unit = [](const Vector3& value) {
        const double length = std::hypot(value[0], value[1], value[2]);
        if (!std::isfinite(length) || length < 1e-12) {
            throw std::runtime_error("BDS PCO: undefined attitude axis");
        }
        return Vector3{value[0] / length, value[1] / length, value[2] / length};
    };
    const Vector3 zAxis = unit({-position[0], -position[1], -position[2]});
    // Form the orbit normal with inertial velocity, expressed in ECEF.
    const Vector3 velocityInertial{
        velocity[0] - constants::Cgcs2000EarthAngularVelocity * position[1],
        velocity[1] + constants::Cgcs2000EarthAngularVelocity * position[0],
        velocity[2],
    };
    const Vector3 orbitY = unit(Cross(zAxis, unit(velocityInertial)));
    const double usedLeapSeconds = leapSeconds > 0 ? leapSeconds : 18;
    const double gpsMjd = GpsEpochMjd + gpsWeek * 7.0 + gpsSec / 86400.0;
    const Vector3 sun = Multiply(ECI_To_ECEF(gpsMjd, usedLeapSeconds),
                                 Sun_Position_ECI(gpsMjd - usedLeapSeconds / 86400.0));
    const Vector3 sunDirection = unit({sun[0] - position[0], sun[1] - position[1],
                                      sun[2] - position[2]});
    if (betaDeg != nullptr) {
        const double sineBeta = -(orbitY[0] * sunDirection[0]
            + orbitY[1] * sunDirection[1] + orbitY[2] * sunDirection[2]);
        *betaDeg = std::asin(std::clamp(sineBeta, -1.0, 1.0)) / DegreesToRadians;
    }
    // GEO: X follows flight, Y is opposite orbit normal, Z points to Earth.
    // Other satellites: nominal yaw steering; special yaw maneuvers are not modeled.
    const Vector3 yAxis = isGeo ? orbitY : unit(Cross(zAxis, sunDirection));
    const Vector3 xAxis = unit(Cross(yAxis, zAxis));
    return {
        xAxis[0] * pco[0] + yAxis[0] * pco[1] + zAxis[0] * pco[2],
        xAxis[1] * pco[0] + yAxis[1] * pco[1] + zAxis[1] * pco[2],
        xAxis[2] * pco[0] + yAxis[2] * pco[1] + zAxis[2] * pco[2],
    };
}

}  // namespace orbclkcmp
