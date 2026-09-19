#pragma once

namespace orbclkcmp::constants {

constexpr int GNum = 32;
constexpr int RNum = 27;
constexpr int CNum = 46;
constexpr int NumE = 36;
constexpr int JNum = 7;

constexpr int TotalSatNum = GNum + RNum + CNum + NumE + JNum;
constexpr int Sp3EpochSlots = 10;
constexpr int ClockEpochSlots = 2;
// SP3 accuracy values use the unified GPS/GLO/BDS/GAL/QZSS satellite index.
constexpr int Sp3OrbitAccuracySlots = TotalSatNum;
constexpr int OutputFileSlots = 50;
constexpr int GlonassNavSlots = 100;

constexpr double SpeedOfLight = 299792458.0;
constexpr double AxisWgs84 = 6378137.0;
constexpr double EccentricitySquared = 0.0066943800229;
constexpr double Pi = 3.1415926535897932;
// WGS84 constants for GPS/QZSS navigation and SP3 earth rotation.
constexpr double Wgs84EarthAngularVelocity = 7.2921151467e-5;
constexpr double Wgs84EarthGravity = 3.986005e14;

// CGCS2000 constants for BDS navigation.
constexpr double Cgcs2000EarthAngularVelocity = 7.292115e-5;
constexpr double Cgcs2000EarthGravity = 3.986004418e14;

// Galileo/GTRF constants for Galileo navigation.
constexpr double GalileoEarthAngularVelocity = 7.2921151467e-5;
constexpr double GalileoEarthGravity = 3.986004418e14;

// GLONASS acceleration constants.
// These values use kilometers.
constexpr double GlonassEarthAngularVelocity = 0.72921151e-4;
constexpr double GlonassEarthGravityKm = 398600.44;
constexpr double GlonassEarthSemiMajorAxisKm = 6378.136;
constexpr double GlonassJ2 = -1082.63e-6;

}  // namespace orbclkcmp::constants
