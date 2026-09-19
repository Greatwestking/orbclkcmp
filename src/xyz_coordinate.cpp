#include "xyz_coordinate.hpp"

#include <cmath>
#include <stdexcept>

#include "constants.hpp"
#include "cross.hpp"

namespace orbclkcmp {

namespace {

double Dot(const Vector3& a, const Vector3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const Vector3& value) {
    return std::sqrt(Dot(value, value));
}

Vector3 Normalized(const Vector3& value) {
    const double length = Norm(value);
    if (length == 0.0) {
        throw std::invalid_argument("cannot normalize a zero vector");
    }
    return {value[0] / length, value[1] / length, value[2] / length};
}

}  // namespace

BLH XYZ2BLH(double x, double y, double z) {
    constexpr double latitudeTolerance = 1.0e-12;
    constexpr double heightTolerance = 1.0e-5;

    double latitude = 0.0;
    double height = 0.0;
    double deltaLatitude = 1.0;
    double deltaHeight = 1.0;

    const double p = std::sqrt(x * x + y * y);
    latitude = std::atan2(z, p / (1.0 - constants::EccentricitySquared));

    while (deltaLatitude > latitudeTolerance || deltaHeight > heightTolerance) {
        const double previousLatitude = latitude;
        const double previousHeight = height;
        const double sinLatitude = std::sin(latitude);
        const double primeVerticalRadius =
            constants::AxisWgs84 /
            std::sqrt(1.0 - constants::EccentricitySquared * sinLatitude * sinLatitude);

        height = p * std::cos(latitude) + z * sinLatitude -
                 (constants::AxisWgs84 * constants::AxisWgs84) / primeVerticalRadius;
        latitude = std::atan2(
            z,
            p * (1.0 - constants::EccentricitySquared * primeVerticalRadius /
                           (primeVerticalRadius + height)));

        deltaLatitude = std::abs(latitude - previousLatitude);
        deltaHeight = std::abs(height - previousHeight);
    }

    double longitudeDeg = std::atan2(y, x) * 180.0 / constants::Pi;
    if (longitudeDeg < 0.0) {
        longitudeDeg += 360.0;
    }

    return BLH{
        latitude * 180.0 / constants::Pi,
        longitudeDeg,
        height,
    };
}

Vector3 XYZ2RTN(const Vector3& satCoor, const Vector3& satVel, const Vector3& dXYZ) {
    // Express inertial orbital velocity in the ECEF axes.
    const double omega = constants::Wgs84EarthAngularVelocity;
    const Vector3 orbitalVelocity{
        satVel[0] - omega * satCoor[1],
        satVel[1] + omega * satCoor[0],
        satVel[2],
    };
    const Vector3 radial = Normalized(satCoor);
    const Vector3 normal = Normalized(Cross(radial, orbitalVelocity));
    const Vector3 track = Cross(normal, radial);

    return {
        Dot(track, dXYZ),
        Dot(normal, dXYZ),
        Dot(radial, dXYZ),
    };
}

}  // namespace orbclkcmp
