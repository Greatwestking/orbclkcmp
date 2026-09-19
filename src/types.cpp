#include "types.hpp"

#include <stdexcept>

namespace orbclkcmp {

Eph::Eph() {
    for (auto& row : coor) {
        row.fill(9999.0);
    }
    clk.fill(999999.999999);
}

SSRData::SSRData() {
    dClk.fill(0.0);
    for (Vector3& value : dOrb) {
        value = {0.0, 0.0, 0.0};
    }
    iode.fill(-2.0);
}

void PCOTable::resize(int frequencyCount) {
    if (frequencyCount < 0) {
        throw std::out_of_range("frequency count must be non-negative");
    }
    values.resize(static_cast<std::size_t>(frequencyCount));
}

int PCOTable::frequencyCount() const {
    return static_cast<int>(values.size());
}

Vector3& PCOTable::at(int Get_Frequency_Index) {
    if (Get_Frequency_Index < 0 || Get_Frequency_Index >= frequencyCount()) {
        throw std::out_of_range("PCO frequency index out of range");
    }
    return values[static_cast<std::size_t>(Get_Frequency_Index)];
}

const Vector3& PCOTable::at(int Get_Frequency_Index) const {
    if (Get_Frequency_Index < 0 || Get_Frequency_Index >= frequencyCount()) {
        throw std::out_of_range("PCO frequency index out of range");
    }
    return values[static_cast<std::size_t>(Get_Frequency_Index)];
}

void PCVTable::resize(int zenithCountValue, int azimuthCountValue, int frequencyCountValue) {
    if (zenithCountValue < 0 || azimuthCountValue < 0 || frequencyCountValue < 0) {
        throw std::out_of_range("PCV dimensions must be non-negative");
    }

    zenithCount = zenithCountValue;
    azimuthCount = azimuthCountValue;
    frequencyCount = frequencyCountValue;

    const auto total =
        static_cast<std::size_t>(zenithCount) *
        static_cast<std::size_t>(azimuthCount) *
        static_cast<std::size_t>(frequencyCount);
    values.assign(total, 0.0);
}

double& PCVTable::at(int zenithIndex, int azimuthIndex, int Get_Frequency_Index) {
    return values[static_cast<std::size_t>(index(zenithIndex, azimuthIndex, Get_Frequency_Index))];
}

const double& PCVTable::at(int zenithIndex, int azimuthIndex, int Get_Frequency_Index) const {
    return values[static_cast<std::size_t>(index(zenithIndex, azimuthIndex, Get_Frequency_Index))];
}

int PCVTable::index(int zenithIndex, int azimuthIndex, int Get_Frequency_Index) const {
    if (zenithIndex < 0 || zenithIndex >= zenithCount) {
        throw std::out_of_range("PCV zenith index out of range");
    }
    if (azimuthIndex < 0 || azimuthIndex >= azimuthCount) {
        throw std::out_of_range("PCV azimuth index out of range");
    }
    if (Get_Frequency_Index < 0 || Get_Frequency_Index >= frequencyCount) {
        throw std::out_of_range("PCV frequency index out of range");
    }

    return zenithIndex +
           zenithCount * (azimuthIndex + azimuthCount * Get_Frequency_Index);
}

}  // namespace orbclkcmp
