#pragma once

#include <array>
#include <iosfwd>
#include <string>
#include <vector>

#include "config.hpp"

namespace orbclkcmp {

struct ProcessDay {
    int year = 0;
    int doy = 0;
    int gpsWeek = 0;
    int gpsDay = 0;
    std::string gpsDayString;
    std::string yearDoyString;
};

Config Read_Config(const std::string& path);
std::string Get_SISRE_FileName(const Config& config, char system, int prn);
std::vector<char> Get_Process_Systems(const Config& config);
int Get_SatNum(char system);
ProcessDay Make_Process_Day(int yearStart, int inputDoy);
ProductFileNames Make_Product_FileNames(const Config& config, const ProcessDay& day);

// Run the main processing workflow.
int Run_Main(const Config& config, std::ostream& out, std::ostream& err);

}  // namespace orbclkcmp
