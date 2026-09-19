#pragma once

#include <array>
#include <string>

#include "types.hpp"

namespace orbclkcmp {

struct ProductFileNames {
    std::array<std::string, 2> sp3;
    std::array<std::string, 2> clk;
    std::array<std::string, 2> nav;
    std::string dcb;
};

struct Config {
    std::string configDir;
    int yearStart = 2024;
    int doyStart = 302;
    int doyEnd = 302;
    bool daySet = false;
    bool dayAuto = true;
    double interval = 30.0;
    std::string Sys = "C";
    std::string AntFile = "D:\\data\\ant\\igs20_2396.atx";
    std::string BrdAntFile;
    std::string brdPCO = "RADIAL";
    std::string DCBDir;
    std::string DCBtype = "CAS0MGXRAP.BSX";
    std::string OutDir;
    std::array<int, 2> orbtype{1, 1};
    std::array<int, 2> clktype{1, 1};
    std::string mode = "both";
    std::array<std::string, 2> productTypes{"final", "final"};
    std::array<std::string, 2> ACs;
    std::array<std::string, 2> orbtypeText{"SP3", "SP3"};
    std::array<std::string, 2> clktypeText{"SP3", "SP3"};
    std::array<BDSFreq, 2> bdsfreq{BDSFreq::B1B3, BDSFreq::B1B3};
    BDSBrdClockCorr bdsBrdClockCorr = BDSBrdClockCorr::DCB;
    std::array<std::string, 2> productDirs;
    ProductFileNames productFiles;
    bool do_orbit = true;
    bool do_clock = true;
    bool modeSet = false;
    bool IsCNAV = false;
    std::string clkdatum = "R-C";
    bool clkdatumSet = false;
    std::string clkdatumMethod = "MED";
    int ssrcorrtype = 0;
    std::string SSRRef;
    std::string SSRFile;
    std::string SSRDir;
    std::string SSRFormat = "YYYYDDD";
    double SSRAge = 30.0;
};

}  // namespace orbclkcmp
