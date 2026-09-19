#pragma once

#include "types.hpp"

namespace orbclkcmp {

struct BLH {
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double heightMeters = 0.0;
};

// Convert WGS84 ECEF XYZ in meters to latitude, longitude, height.
BLH XYZ2BLH(double x, double y, double z);

// Project an ECEF difference onto orthonormal T/N/R axes. satVel is ECEF velocity.
Vector3 XYZ2RTN(const Vector3& satCoor, const Vector3& satVel, const Vector3& dXYZ);

}  // namespace orbclkcmp
