#include "lagrange.hpp"

#include <cmath>
#include <stdexcept>

namespace orbclkcmp {

namespace {

bool Is_Zero_Sample(const Vector3& value) {
    return std::abs(value[0]) < 1.0e-8 &&
           std::abs(value[1]) < 1.0e-8 &&
           std::abs(value[2]) < 1.0e-8;
}

void Validate_Input_Sizes(const std::vector<double>& x, const std::vector<Vector3>& y) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("Lagrange x and y sizes must match");
    }
}

Vector3 Invalid_Vector() {
    return {9999.0, 9999.0, 9999.0};
}

}  // namespace

Vector3 Lagrange(const std::vector<double>& x, const std::vector<Vector3>& y, double xh) {
    Validate_Input_Sizes(x, y);

    std::vector<double> validX;
    std::vector<Vector3> validY;
    validX.reserve(x.size());
    validY.reserve(y.size());

    for (std::size_t i = 0; i < x.size(); ++i) {
        if (Is_Zero_Sample(y[i])) {
            continue;
        }
        validX.push_back(x[i]);
        validY.push_back(y[i]);
    }

    if (validX.size() < 6) {
        return Invalid_Vector();
    }

    Vector3 yh{};
    for (std::size_t i = 0; i < validX.size(); ++i) {
        double p = 1.0;
        for (std::size_t j = 0; j < validX.size(); ++j) {
            if (i == j) {
                continue;
            }
            p *= (xh - validX[j]) / (validX[i] - validX[j]);
        }

        for (int component = 0; component < 3; ++component) {
            yh[component] += validY[i][component] * p;
        }
    }

    return yh;
}

Vector3 Lagrange_Vel(const std::vector<double>& x, const std::vector<Vector3>& y, double xh) {
    Validate_Input_Sizes(x, y);

    std::vector<double> validX;
    std::vector<Vector3> validY;
    validX.reserve(x.size());
    validY.reserve(y.size());

    for (std::size_t i = 0; i < x.size(); ++i) {
        if (Is_Zero_Sample(y[i])) {
            continue;
        }
        validX.push_back(x[i]);
        validY.push_back(y[i]);
    }

    if (validX.size() < 6) {
        return Invalid_Vector();
    }

    Vector3 yh{};
    for (std::size_t i = 0; i < validX.size(); ++i) {
        double p = 0.0;
        double coefficient = 1.0;
        for (std::size_t j = 0; j < validX.size(); ++j) {
            if (i == j) {
                continue;
            }

            double q = 1.0;
            for (std::size_t k = 0; k < validX.size(); ++k) {
                if (k == j || k == i) {
                    continue;
                }
                q *= (xh - validX[k]);
            }

            p += q;
            coefficient *= (validX[i] - validX[j]);
        }

        for (int component = 0; component < 3; ++component) {
            yh[component] += validY[i][component] * p / coefficient;
        }
    }

    return yh;
}

}  // namespace orbclkcmp
