#pragma once

#include "types.hpp"

namespace orbclkcmp {

// Convert a satellite antenna PCO from satellite-fixed NEU axes to ECEF.
Vector3 Satellite_PCO_To_ECEF(int gpsWeek, double gpsSec, int leapSeconds,
                              const Vector3& satellitePosition,
                              const Vector3& pco);

// Convert BDS body XYZ PCO to ECEF using nominal GEO/IGSO/MEO attitude.
// velocity is an ECEF derivative. betaDeg reports Sun elevation above the orbit.
Vector3 BDS_PCO_To_ECEF(int gpsWeek, double gpsSec, int leapSeconds,
                        const Vector3& position, const Vector3& velocity,
                        const Vector3& pco, bool isGeo, double* betaDeg = nullptr);

}  // namespace orbclkcmp
