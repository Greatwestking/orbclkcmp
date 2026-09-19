#pragma once

#include <array>
#include <string>
#include <vector>

#include "constants.hpp"

namespace orbclkcmp {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;

enum class BDSFreq {
    B3,
    B1B3,
    B1B2,
};

enum class BDSBrdClockCorr {
    DCB,
    TGD,
};

// GLONASS frequency channel number for each satellite.
using GLOFrequencyChannels = std::array<int, constants::RNum>;

// Runtime constants loaded or set during processing.
struct RuntimeConstants {
    int leapSeconds = 0;
    std::array<Vector3, constants::CNum> tgdBias{};
    std::array<std::array<double, 2>, constants::CNum> newTgd{};
};

// File-unit IDs for I/O.
struct FileIds {
    int nextFileId = 20;
    std::array<int, constants::OutputFileSlots> outId{};
    int navId = 0;
    int navIdR = 0;
    int navIdE = 0;
    std::array<int, 2> navIdC{};
    std::array<int, 2> sp3Id{};
    std::array<int, 2> clkId{};
    int dcbId = 0;
    int tgdBiasId = 0;
    int antId = 0;
};

// Earth orientation parameters.
struct EOP {
    std::array<double, 2> mjd{};
    std::array<double, 2> x{};
    std::array<double, 2> y{};
    std::array<double, 2> dut1{};
    std::array<double, 2> dx{};
    std::array<double, 2> dy{};
};

// Mean pole model parameters.
struct MeanPole {
    double xp0 = 0.054;
    double yp0 = 0.357;
    double xpRate = 0.00083;
    double ypRate = 0.00395;
    double tref = 51544.0;
};

// Rotation matrices for coordinate transforms.
struct RotationState {
    Matrix3 satelliteToCrs{};
    Matrix3 trsToCrs{};
    Matrix3 crsToTrs{};
    Matrix3 xyzToNeu{};
};

// Navigation header fields.
struct NavHead {
    int version = 2;
    int leapSeconds = 0;
    std::array<double, 4> alpha{};
    std::array<double, 4> beta{};
};

// SP3 header fields.
struct Sp3Head {
    int prnCount = 0;
    std::vector<int> prn;
    std::vector<char> system;
    int gpsWeek = 0;
    double gpsSec = 0.0;
    double sp3Time = 0.0;
    std::array<int, constants::Sp3OrbitAccuracySlots> orbitAccuracy{};
};

// SP3 coordinates and clock samples for one satellite.
struct Eph {
    std::array<std::array<double, constants::Sp3EpochSlots>, 3> coor{};
    std::array<double, constants::Sp3EpochSlots> clk{};

    Eph();
};

// SP3 epochs and per-satellite values.
struct Sp3Data {
    std::array<int, constants::Sp3EpochSlots> gpsWeek{};
    std::array<double, constants::Sp3EpochSlots> gpsSec{};
    std::array<Eph, constants::TotalSatNum> eph{};
};

// Clock values for one satellite.
struct ClockRecord {
    std::array<double, constants::ClockEpochSlots> clk{9999.0, 9999.0};
    double clkVel = 9999.0;
};

// Clock epochs and per-satellite clock values.
struct ClockData {
    std::array<int, constants::ClockEpochSlots> gpsWeek{};
    std::array<double, constants::ClockEpochSlots> gpsSec{};
    std::array<ClockRecord, constants::TotalSatNum> as{};
};

// SSR orbit, clock, and IODE corrections.
struct SSRData {
    int week = 0;
    double sow = 0.0;
    bool isBNC = false;
    std::array<double, constants::TotalSatNum> dClk{};
    std::array<Vector3, constants::TotalSatNum> dOrb{};
    std::array<double, constants::TotalSatNum> iode{};

    SSRData();
};

// Broadcast ephemeris record for GPS/BDS/Galileo/QZSS.
struct NavRecord {
    int gpsWeek = 0;
    double gpsSec = 0.0;
    double a0 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    double aDot = 0.0;
    double iode = 0.0;
    double crs = 0.0;
    double deltaN = 0.0;
    double m0 = 0.0;
    double cuc = 0.0;
    double e = 0.0;
    double cus = 0.0;
    double sqrtA = 0.0;
    double toe = 0.0;
    double cic = 0.0;
    double omega0 = 0.0;
    double cis = 0.0;
    double i0 = 0.0;
    double crc = 0.0;
    double omega = 0.0;
    double omegaDot = 0.0;
    double idot = 0.0;
    double nDot = 0.0;
    double code = 0.0;
    double weekNo = 0.0;
    double health = 0.0;
    std::array<double, 2> tgd{};
    double iodc = 0.0;
    double iscL1Ca = 0.0;
    double iscL2C = 0.0;
    double iscL5I5 = 0.0;
    double iscL5Q5 = 0.0;
};

// Broadcast ephemeris records for one satellite.
struct NavData {
    std::vector<NavRecord> records;
};

// GLONASS broadcast ephemeris record.
struct GlonassNavRecord {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double sec = 0.0;
    int gpsWeek = 0;
    double gpsSec = 0.0;
    double a0 = 0.0;
    double a1 = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double health = 0.0;
    double frequency = 0.0;
    double tgd = 0.0;
};

// GLONASS ephemeris records for one satellite.
struct GlonassNavData {
    std::array<GlonassNavRecord, constants::GlonassNavSlots> records{};
};

// PCO values, one XYZ vector per frequency.
struct PCOTable {
    std::vector<Vector3> values;

    void resize(int frequencyCount);
    int frequencyCount() const;
    Vector3& at(int Get_Frequency_Index);
    const Vector3& at(int Get_Frequency_Index) const;
};

// PCV values indexed by zenith, azimuth, and frequency.
struct PCVTable {
    int zenithCount = 0;
    int azimuthCount = 0;
    int frequencyCount = 0;
    std::vector<double> values;

    void resize(int zenithCountValue, int azimuthCountValue, int frequencyCountValue);
    double& at(int zenithIndex, int azimuthIndex, int Get_Frequency_Index);
    const double& at(int zenithIndex, int azimuthIndex, int Get_Frequency_Index) const;

private:
    int index(int zenithIndex, int azimuthIndex, int Get_Frequency_Index) const;
};

// Antenna type, PRN/SVN, PCO table, and PCV table.
struct Antenna {
    std::string antennaType;
    int prn = 0;
    int svn = 0;
    int validFromYearDoy = 0;   // ATX VALID FROM date, YYYYDDD.
    int validUntilYearDoy = 0;  // ATX VALID UNTIL date, YYYYDDD.
    int zenithCount = 0;
    int azimuthCount = 0;
    double azimuthStep = 0.0;
    double zenithStep = 0.0;
    std::vector<double> zenith;
    std::vector<double> azimuth;
    std::vector<std::string> frequency;
    PCOTable pco;
    PCVTable pcv;
};

// Runtime switches for navigation processing.
struct RuntimeOptions {
    bool IsCNAV = false;           // Read CNAV-specific navigation fields.
};

// Shared processing state.
struct SharedState {
    RuntimeConstants constants;
    GLOFrequencyChannels gloFrequencyChannels{};
    FileIds fileIds;
    EOP eop;
    MeanPole meanPole;
    RotationState rotation;
    NavHead navHead;
    NavHead navHeadR;
    NavHead navHeadE;
    std::array<NavData, constants::GNum> navDataGps;
    std::array<NavData, constants::NumE> navDataGalileo;
    std::array<NavData, constants::CNum> navDataBds;
    std::array<NavData, constants::JNum> navDataQzss;
    std::array<GlonassNavData, constants::RNum> navDataGlonass;
    std::array<Antenna, constants::TotalSatNum> antennas;
    RuntimeOptions options;
};

// Satellite position, velocity, clock, and relativity.
struct SatState {
    Vector3 position{};
    Vector3 velocity{};
    double clock = 0.0;
    double relativity = 0.0;
};

}  // namespace orbclkcmp
