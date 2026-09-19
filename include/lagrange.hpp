#pragma once

#include <vector>

#include "types.hpp"

namespace orbclkcmp {

// Interpolate a 3D value at xh.
Vector3 Lagrange(const std::vector<double>& x, const std::vector<Vector3>& y, double xh);

// Calculate the Lagrange interpolation derivative at xh.
Vector3 Lagrange_Vel(const std::vector<double>& x, const std::vector<Vector3>& y, double xh);

}  // namespace orbclkcmp
