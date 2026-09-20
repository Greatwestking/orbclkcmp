#include "main.hpp"

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "cal_sat_clk.hpp"
#include "cal_sat_pos_nav.hpp"
#include "cal_sat_pos_sp3.hpp"
#include "constants.hpp"
#include "get_ant.hpp"
#include "prn_index.hpp"
#include "read_clk.hpp"
#include "read_mgex_dcb.hpp"
#include "read_nav.hpp"
#include "read_sp3.hpp"
#include "satellite_pco.hpp"
#include "ssr.hpp"
#include "time_convert.hpp"
#include "xyz_coordinate.hpp"

namespace orbclkcmp {
namespace {

constexpr int BDS_PRN_Renumbering_YearDoy = 2026100;  // 2026-04-10.
constexpr int BDS_PRN_Renumbering_EndYearDoy = 2026110;  // 2026-04-20.
constexpr double BDS2_IPCO = 1.27;

enum class BDSOrbitType {
    Unknown,
    GEO,
    IGSO,
    MEO,
};

int Read_Int(const std::string& value, const std::string& option) {
    // Read an integer.
    try {
        std::size_t pos = 0;
        const int result = std::stoi(value, &pos);
        if (pos != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::invalid_argument(option + " expects an integer: " + value);
    }
}

double Read_Double(const std::string& value, const std::string& option) {
    // Read a real number.
    try {
        std::size_t pos = 0;
        const double result = std::stod(value, &pos);
        if (pos != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::invalid_argument(option + " expects a number: " + value);
    }
}

void Set_Mode(Config& config) {
    // Set do_orbit and do_clock from config.mode.
    // Convert mode to lowercase.
    std::transform(config.mode.begin(), config.mode.end(), config.mode.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (config.mode == "orb" || config.mode == "orbit") {
        config.mode = "orb";
        config.do_orbit = true;
        config.do_clock = false;
    } else if (config.mode == "clk" || config.mode == "clock") {
        config.mode = "clk";
        config.do_orbit = false;
        config.do_clock = true;
    } else if (config.mode == "both") {
        config.mode = "both";
        config.do_orbit = true;
        config.do_clock = true;
    } else {
        throw std::invalid_argument("mode expects orb, clk, or both: " + config.mode);
    }
}

std::string trim(const std::string& value) {
    // Remove spaces at both ends.
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool Read_Bool(const std::string& value, const std::string& option) {
    // Read true/false text.
    const std::string text = trim(value);
    if (text == "1" || text == "true" || text == "TRUE" || text == ".true.") {
        return true;
    }
    if (text == "0" || text == "false" || text == "FALSE" || text == ".false.") {
        return false;
    }
    throw std::invalid_argument(option + " expects true/false: " + value);
}

std::string Upper_Text(std::string value) {
    // Convert text to uppercase.
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

std::string Lower_Text(std::string value) {
    // Convert text to lowercase.
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

int Year_Doy_Number(const ProcessDay& day) {
    // Return YYYYDDD for the current processing day.
    return day.year * 1000 + day.doy;
}

std::string First_Word(const std::string& value) {
    // Return the first word.
    std::istringstream input(value);
    std::string word;
    input >> word;
    return word;
}

std::string Text_After_First_Word(const std::string& value) {
    // Return text after the first word.
    std::istringstream input(value);
    std::string word;
    input >> word;
    std::string rest;
    std::getline(input, rest);
    return trim(rest);
}

std::string Read_Clkdatum(const std::string& value, const std::string& option) {
    // Read clkdatum (from initial clock bias).
    std::string text = Upper_Text(First_Word(value));
    std::replace(text.begin(), text.end(), '_', '-');
    if (text == "CLK") {
        return "CLK";
    }
    if (text == "R-C") {
        return "R-C";
    }
    throw std::invalid_argument(option + " expects CLK or R-C: " + value);
}

std::string Read_Clkdatum_Method(const std::string& value, const std::string& option) {
    // Read clkdatum method (from initial clock bias).
    const std::string text = Upper_Text(First_Word(value));
    if (text == "MED" || text == "MEDIAN") {
        return "MED";
    }
    if (text == "CMED" || text == "CMEDIAN" || text == "CONTINUOUS_MEDIAN") {
        return "CMED";
    }
    if (text == "CAVG" || text == "CAVERAGE" || text == "CONTINUOUS_AVERAGE") {
        return "CAVG";
    }
    throw std::invalid_argument(option + " expects MED, CMED, or CAVG: " + value);
}

int Read_ssrcorrtype(const std::string& value, const std::string& option) {
    // Read SSR correction type.
    const std::string text = Upper_Text(First_Word(value));
    if (text == "0" || text == "NONE" || text == "NO") {
        return 0;
    }
    if (text == "1" || text == "ORB" || text == "ORBIT") {
        return 1;
    }
    if (text == "2" || text == "SSR" || text == "BNC") {
        return 2;
    }
    throw std::invalid_argument(option + " expects 0, 1, or 2: " + value);
}

std::string Read_SSRRef(const std::string& value, const std::string& option) {
    // Read SSR orbit reference.
    std::string text = Upper_Text(First_Word(value));
    if (text.empty() || text == "NONE" || text == "NO") {
        return "";
    }
    text.erase(std::remove(text.begin(), text.end(), '-'), text.end());
    text.erase(std::remove(text.begin(), text.end(), '_'), text.end());
    if (text == "APCB1B2" || text == "APCB1CB2A") {
        return "APCB1B2";
    }
    if (text == "APCB1B3" || text == "APCB1IB3I") {
        return "APCB1B3";
    }
    if (text == "B1IAPC" || text == "APCL1") {
        return "B1IAPC";
    }
    if (text == "IFTOL1" || text == "APC1L") {
        return "IFTOL1";
    }
    if (text == "APC" || text == "COM" || text == "APCPC") {
        return text;
    }
    throw std::invalid_argument(option + " expects APC, COM, APCPC, APC_B1B2, APC_B1B3, B1I_APC, or IF_TO_L1: " + value);
}

BDSFreq Read_BDSFreq(const std::string& value, const std::string& option) {
    // Read BDS frequency combination.
    std::string text = Upper_Text(First_Word(value));
    std::replace(text.begin(), text.end(), '-', '_');
    if (text == "B3" || text == "B3I") {
        return BDSFreq::B3;
    }
    if (text == "B1B3" || text == "B1I_B3I") {
        return BDSFreq::B1B3;
    }
    if (text == "B1B2" || text == "B1C_B2A" || text == "B1C_B2") {
        return BDSFreq::B1B2;
    }
    throw std::invalid_argument(option + " expects B3, B1B3, or B1B2: " + value);
}

BDSBrdClockCorr Read_BDS_Brd_Clock_Corr(const std::string& value, const std::string& option) {
    // Read the correction source for a BDS broadcast clock.
    const std::string text = Upper_Text(First_Word(value));
    if (text == "DCB") {
        return BDSBrdClockCorr::DCB;
    }
    if (text == "TGD") {
        return BDSBrdClockCorr::TGD;
    }
    throw std::invalid_argument(option + " expects DCB or TGD: " + value);
}

std::string BDS_Brd_Clock_Corr_Text(BDSBrdClockCorr correction) {
    // Return the correction source for a BDS broadcast clock.
    return correction == BDSBrdClockCorr::TGD ? "TGD" : "DCB";
}

std::string BDSFreq_Text(BDSFreq freq) {
    // Return BDS frequency combination text.
    if (freq == BDSFreq::B3) {
        return "B3";
    }
    return freq == BDSFreq::B1B2 ? "B1B2" : "B1B3";
}

std::string Read_DCBtype(const std::string& value, const std::string& option) {
    // Read DCBtype as PREFIX.EXT.
    const std::string text = Upper_Text(First_Word(value));
    const std::size_t dot = text.find('.');
    if (dot == std::string::npos || dot == 0 || dot + 1 >= text.size()) {
        throw std::invalid_argument(option + " expects PREFIX.EXT, for example CAS0OPSRAP.BIA");
    }
    return text;
}

std::vector<std::string> Split_Words(const std::string& value) {
    // Split text into words.
    std::istringstream input(value);
    std::vector<std::string> result;
    std::string word;
    while (input >> word) {
        result.push_back(word);
    }
    return result;
}

int Read_orbtype(const std::string& value, const std::string& option) {
    // Return the orbit type code.
    const std::string text = Upper_Text(First_Word(value));
    if (text == "SP3") {
        return 1;
    }
    if (text == "BRD4" || text == "BRDM" || text == "BRD" || text == "NAV") {
        return 2;
    }
    throw std::invalid_argument(option + " expects SP3, BRD4, or BRDM: " + value);
}

int Read_clktype(const std::string& value, const std::string& option) {
    // Return the clock type code.
    const std::string text = Upper_Text(First_Word(value));
    if (text == "BRD4" || text == "BRDM" || text == "BRD") {
        return 0;
    }
    if (text == "CLK") {
        return 1;
    }
    if (text == "SP3") {
        return 2;
    }
    throw std::invalid_argument(option + " expects SP3, CLK, BRD4, or BRDM: " + value);
}

std::string Read_Product_Type_Text(const std::string& value, const std::string& option, bool allowClk) {
    // Return the product type text.
    const std::string text = Upper_Text(First_Word(value));
    if (text == "SP3" || text == "BRD4" || text == "BRDM" || (allowClk && text == "CLK")) {
        return text;
    }
    throw std::invalid_argument(option + " expects SP3, " + std::string(allowClk ? "CLK, " : "") + "BRD4, or BRDM: " + value);
}

void Set_orbtype(Config& config, int product, const std::string& value, const std::string& key) {
    // Set orbtype for one product.
    config.orbtypeText[static_cast<std::size_t>(product)] = Read_Product_Type_Text(value, key, false);
    if (product == 0) {
        config.orbtype[0] = Read_orbtype(value, key);
    } else {
        config.orbtype[1] = Read_orbtype(value, key);
    }
}

void Set_clktype(Config& config, int product, const std::string& value, const std::string& key) {
    // Set clktype for one product.
    config.clktypeText[static_cast<std::size_t>(product)] = Read_Product_Type_Text(value, key, true);
    if (product == 0) {
        config.clktype[0] = Read_clktype(value, key);
    } else {
        config.clktype[1] = Read_clktype(value, key);
    }
}

void Set_Orbit_Product(Config& config, int product, const std::string& value, const std::string& key) {
    // Set orbit type and orbit file for one product.
    const int orbtypeValue = Read_orbtype(value, key);
    config.orbtypeText[static_cast<std::size_t>(product)] = Read_Product_Type_Text(value, key, false);
    const std::string fileName = Text_After_First_Word(value);
    if (product == 0) {
        config.orbtype[0] = orbtypeValue;
    } else {
        config.orbtype[1] = orbtypeValue;
    }

    if (fileName.empty()) {
        return;
    }
    if (orbtypeValue == 1) {
        config.productFiles.sp3[static_cast<std::size_t>(product)] = fileName;
    } else {
        config.productFiles.nav[static_cast<std::size_t>(product)] = fileName;
    }
}

void Set_Clock_Product(Config& config, int product, const std::string& value, const std::string& key) {
    // Set clock type and clock file for one product.
    const int clktypeValue = Read_clktype(value, key);
    config.clktypeText[static_cast<std::size_t>(product)] = Read_Product_Type_Text(value, key, true);
    const std::string fileName = Text_After_First_Word(value);
    if (product == 0) {
        config.clktype[0] = clktypeValue;
    } else {
        config.clktype[1] = clktypeValue;
    }

    if (fileName.empty()) {
        return;
    }
    if (clktypeValue == 1) {
        config.productFiles.clk[static_cast<std::size_t>(product)] = fileName;
    } else if (clktypeValue == 2) {
        config.productFiles.sp3[static_cast<std::size_t>(product)] = fileName;
    } else {
        config.productFiles.nav[static_cast<std::size_t>(product)] = fileName;
    }
}

void Set_Common_Clock_Product(Config& config, const std::string& value, const std::string& key) {
    // Set clock type and files for both products.
    const int clktypeValue = Read_clktype(value, key);
    const std::string clktypeName = Read_Product_Type_Text(value, key, true);
    config.clktype[0] = clktypeValue;
    config.clktype[1] = clktypeValue;
    config.clktypeText[0] = clktypeName;
    config.clktypeText[1] = clktypeName;

    const std::vector<std::string> tokens = Split_Words(value);
    if (clktypeValue == 1) {
        if (tokens.size() >= 2) {
            config.productFiles.clk[0] = tokens[1];
        }
        if (tokens.size() >= 3) {
            config.productFiles.clk[1] = tokens[2];
        }
    } else if (clktypeValue == 2) {
        if (tokens.size() >= 2) {
            config.productFiles.sp3[0] = tokens[1];
        }
        if (tokens.size() >= 3) {
            config.productFiles.sp3[1] = tokens[2];
        }
    } else {
        if (tokens.size() >= 2) {
            config.productFiles.nav[0] = tokens[1];
        }
        if (tokens.size() >= 3) {
            config.productFiles.nav[1] = tokens[2];
        }
    }
}

bool Has_Ending(const std::string& value, const std::string& ending) {
    // Check whether text ends with a suffix.
    if (value.size() < ending.size()) {
        return false;
    }
    return value.compare(value.size() - ending.size(), ending.size(), ending) == 0;
}

void Set_Product_File_By_Name(Config& config, int product, const std::string& fileName) {
    // Set the product file slot from the file name.
    const std::string lowerName = Lower_Text(fileName);
    const std::size_t index = static_cast<std::size_t>(product);

    if (Has_Ending(lowerName, ".sp3")) {
        if (product == 0) {
            config.orbtype[0] = 1;
        } else {
            config.orbtype[1] = 1;
        }
        config.orbtypeText[index] = "SP3";
        config.productFiles.sp3[index] = fileName;
        return;
    }

    if (Has_Ending(lowerName, ".clk")) {
        if (product == 0) {
            config.clktype[0] = 1;
        } else {
            config.clktype[1] = 1;
        }
        config.clktypeText[index] = "CLK";
        config.productFiles.clk[index] = fileName;
        return;
    }

    const std::string brdtype = lowerName.rfind("brd4", 0) == 0 ? "BRD4" : "BRDM";
    if (product == 0) {
        config.orbtype[0] = 2;
        config.orbtypeText[0] = brdtype;
        if (config.clktype[0] != 1) {
            config.clktype[0] = 0;
            config.clktypeText[0] = brdtype;
        }
    } else {
        config.orbtype[1] = 2;
        config.orbtypeText[1] = brdtype;
        if (config.clktype[1] != 1) {
            config.clktype[1] = 0;
            config.clktypeText[1] = brdtype;
        }
    }
    config.productFiles.nav[index] = fileName;
}

void Set_Product_Files(Config& config, int product, const std::string& value, const std::string& key) {
    // Read explicit product files for one product.
    const std::vector<std::string> files = Split_Words(value);
    if (files.empty()) {
        throw std::invalid_argument(key + " expects one or more product file names");
    }
    for (const std::string& fileName : files) {
        Set_Product_File_By_Name(config, product, fileName);
    }

    const std::size_t index = static_cast<std::size_t>(product);
    if (!config.productFiles.sp3[index].empty() && config.productFiles.clk[index].empty()) {
        if (product == 0) {
            config.clktype[0] = 2;
        } else {
            config.clktype[1] = 2;
        }
        config.clktypeText[index] = "SP3";
    }
}

std::string Resolve_Config_Path(const Config& config, const std::string& path) {
    // Resolve a path relative to cmp.conf.
    if (path.empty()) {
        return "";
    }
    const std::filesystem::path filePath(path);
    if (filePath.is_absolute() || config.configDir.empty()) {
        return filePath.generic_string();
    }
    return (std::filesystem::path(config.configDir) / filePath).generic_string();
}

void Set_Product_Dir(Config& config, int product, const std::string& value) {
    // Set the product search directory.
    config.productDirs[static_cast<std::size_t>(product)] = Resolve_Config_Path(config, value);
}

void Resolve_Config_File_Paths(Config& config) {
    // Resolve all paths from cmp.conf.
    for (std::string& path : config.productFiles.sp3) {
        path = Resolve_Config_Path(config, path);
    }
    for (std::string& path : config.productFiles.clk) {
        path = Resolve_Config_Path(config, path);
    }
    for (std::string& path : config.productFiles.nav) {
        path = Resolve_Config_Path(config, path);
    }
    config.AntFile = Resolve_Config_Path(config, config.AntFile);
    config.BrdAntFile = Resolve_Config_Path(config, config.BrdAntFile);
    config.DCBDir = Resolve_Config_Path(config, config.DCBDir);
    config.OutDir = Resolve_Config_Path(config, config.OutDir);
    config.SSRFile = Resolve_Config_Path(config, config.SSRFile);
    config.SSRDir = Resolve_Config_Path(config, config.SSRDir);
}

bool Is_Leap_Year(int year) {
    // Check whether the year is a leap year.
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int Days_In_Year(int year) {
    // Return the number of days in the year.
    return Is_Leap_Year(year) ? 366 : 365;
}

bool Valid_Year_Doy(int year, int doy) {
    // Check the YYYY/DOY range.
    return year >= 1901 && year <= 2099 && doy >= 1 && doy <= Days_In_Year(year);
}

bool Infer_Year_Doy_Token(const std::string& token, int& year, int& doy) {
    // Read YYYYDDD from a file name.
    for (std::size_t start = 0; start < token.size(); ++start) {
        if (!std::isdigit(static_cast<unsigned char>(token[start]))) {
            continue;
        }
        std::size_t end = start;
        while (end < token.size() && std::isdigit(static_cast<unsigned char>(token[end]))) {
            ++end;
        }
        const std::string digits = token.substr(start, end - start);
        if (digits.size() >= 7) {
            for (std::size_t pos = 0; pos + 7 <= digits.size(); ++pos) {
                const int candidateYear = Read_Int(digits.substr(pos, 4), "date in file name");
                const int candidateDoy = Read_Int(digits.substr(pos + 4, 3), "date in file name");
                if (Valid_Year_Doy(candidateYear, candidateDoy)) {
                    year = candidateYear;
                    doy = candidateDoy;
                    return true;
                }
            }
        }
        start = end;
    }
    return false;
}

bool Infer_GPSWeek_Day_Token(const std::string& token, int& gpsWeek, int& gpsDay) {
    // Read WWWWD from a file name.
    for (std::size_t start = 0; start < token.size(); ++start) {
        if (!std::isdigit(static_cast<unsigned char>(token[start]))) {
            continue;
        }
        std::size_t end = start;
        while (end < token.size() && std::isdigit(static_cast<unsigned char>(token[end]))) {
            ++end;
        }
        const std::string digits = token.substr(start, end - start);
        if (digits.size() >= 5) {
            for (std::size_t pos = 0; pos + 5 <= digits.size(); ++pos) {
                const int candidateWeek = Read_Int(digits.substr(pos, 4), "GPS week/day in file name");
                const int candidateDay = Read_Int(digits.substr(pos + 4, 1), "GPS week/day in file name");
                if (candidateWeek >= 0 && candidateWeek <= 9999 && candidateDay >= 0 && candidateDay <= 6) {
                    gpsWeek = candidateWeek;
                    gpsDay = candidateDay;
                    return true;
                }
            }
        }
        start = end;
    }
    return false;
}

bool GPSWeek_Day_To_Year_Doy(int gpsWeek, int gpsDay, int& year, int& doy) {
    // Convert GPS week/day to year/DOY.
    for (int candidateYear = 1980; candidateYear <= 2099; ++candidateYear) {
        for (int candidateDoy = 1; candidateDoy <= Days_In_Year(candidateYear); ++candidateDoy) {
            const Gpst gpst = Day2GPST(candidateYear, candidateDoy);
            if (gpst.week == gpsWeek && gpst.day == gpsDay) {
                year = candidateYear;
                doy = candidateDoy;
                return true;
            }
        }
    }
    return false;
}

std::vector<std::string> All_Product_Files(const Config& config) {
    // List explicit product file names.
    std::vector<std::string> files;
    for (const auto& fileName : config.productFiles.sp3) {
        if (!fileName.empty()) {
            files.push_back(fileName);
        }
    }
    for (const auto& fileName : config.productFiles.clk) {
        if (!fileName.empty()) {
            files.push_back(fileName);
        }
    }
    for (const auto& fileName : config.productFiles.nav) {
        if (!fileName.empty()) {
            files.push_back(fileName);
        }
    }
    return files;
}

void Infer_Day(Config& config) {
    // Set the processing day from product file names.
    if (config.daySet && !config.dayAuto) {
        return;
    }

    const std::vector<std::string> files = All_Product_Files(config);
    for (const std::string& fileName : files) {
        int year = 0;
        int doy = 0;
        if (Infer_Year_Doy_Token(fileName, year, doy)) {
            config.yearStart = year;
            config.doyStart = doy;
            config.doyEnd = doy;
            return;
        }
    }

    for (const std::string& fileName : files) {
        int gpsWeek = 0;
        int gpsDay = 0;
        int year = 0;
        int doy = 0;
        if (Infer_GPSWeek_Day_Token(fileName, gpsWeek, gpsDay) && GPSWeek_Day_To_Year_Doy(gpsWeek, gpsDay, year, doy)) {
            config.yearStart = year;
            config.doyStart = doy;
            config.doyEnd = doy;
            return;
        }
    }

    if (config.dayAuto) {
        throw std::invalid_argument("day auto failed: no YYYYDDD or GPS week/day found in product file names");
    }
}

void Require_FileName(const std::string& value, const std::string& key) {
    // Require a file name when no directory is set.
    if (trim(value).empty()) {
        throw std::invalid_argument("missing required config file: " + key);
    }
}

void Check_product_type_match(const Config& config, int product) {
    // Check orbit/clock type rules for one product.
    const std::size_t index = static_cast<std::size_t>(product);
    const std::string name = product == 0 ? "reference" : "compared";
    const std::string orbtypeName = config.orbtypeText[index];
    const std::string clktypeName = config.clktypeText[index];

    if (product == 0 && (orbtypeName == "BRD4" || orbtypeName == "BRDM"
        || clktypeName == "BRD4" || clktypeName == "BRDM")) {
        throw std::invalid_argument("reference must use SP3 orbit with SP3 or CLK clock");
    }

    if (!config.do_clock) {
        return;
    }

    if (config.do_orbit && (clktypeName == "BRD4" || clktypeName == "BRDM") && orbtypeName != clktypeName) {
        throw std::invalid_argument(name + " clock " + clktypeName + " must match " + name + "_orbit");
    }
    if (config.do_orbit && (orbtypeName == "BRD4" || orbtypeName == "BRDM") && clktypeName != orbtypeName) {
        throw std::invalid_argument(name + " orbit " + orbtypeName + " must match " + name + "_clock");
    }
    if (config.do_orbit && clktypeName == "CLK" && orbtypeName != "SP3") {
        throw std::invalid_argument(name + " clock CLK requires " + name + "_orbit SP3");
    }
}

void Check_config(const Config& config) {
    // Check config before processing.
    Check_product_type_match(config, 0);
    Check_product_type_match(config, 1);

    if (config.IsCNAV) {
        for (int product = 0; product < 2; ++product) {
            const std::size_t index = static_cast<std::size_t>(product);
            const bool usesBRDM = (config.do_orbit && config.orbtypeText[index] == "BRDM")
                || (config.do_clock && config.clktypeText[index] == "BRDM");
            if (usesBRDM) {
                throw std::invalid_argument(
                    "is_cnav true requires BRD4; BRDM contains no CNAV message labels");
            }
        }
    }

    for (int product = 0; product < 2; ++product) {
        const int number = product + 1;
        const std::string suffix = std::to_string(number);
        const bool hasProductDir = !config.productDirs[static_cast<std::size_t>(product)].empty();

        if (config.do_orbit) {
            if ((product == 0 ? config.orbtype[0] : config.orbtype[1]) == 1) {
                if (!hasProductDir) {
                    Require_FileName(config.productFiles.sp3[static_cast<std::size_t>(product)], "sp3dir" + suffix);
                }
            } else {
                if (!hasProductDir) {
                    Require_FileName(config.productFiles.nav[static_cast<std::size_t>(product)], "navfile" + suffix);
                }
            }
        }

        if (config.do_clock) {
            const int clktypeValue = product == 0 ? config.clktype[0] : config.clktype[1];
            if (clktypeValue == 1) {
                if (!hasProductDir) {
                    Require_FileName(config.productFiles.clk[static_cast<std::size_t>(product)], "clkdir" + suffix);
                }
            } else if (clktypeValue == 2) {
                if (!hasProductDir) {
                    Require_FileName(config.productFiles.sp3[static_cast<std::size_t>(product)], "sp3dir" + suffix);
                }
            } else {
                if (!hasProductDir) {
                    Require_FileName(config.productFiles.nav[static_cast<std::size_t>(product)], "navfile" + suffix);
                }
            }
        }
    }

    if (config.OutDir.empty()) {
        throw std::invalid_argument("missing required config key: outdir");
    }
    if (config.Sys.empty()) {
        throw std::invalid_argument("missing required config key: systemused");
    }
    for (const char system : config.Sys) {
        if (std::string("GRECJ").find(system) == std::string::npos) {
            throw std::invalid_argument("systemused supports only G, R, E, C, and J");
        }
    }
    if (config.mode != "both" && config.clkdatumSet) {
        throw std::invalid_argument("clkdatum is only valid when mode is both");
    }
    if (!config.SSRRef.empty() && config.ssrcorrtype == 0) {
        throw std::invalid_argument("ssr_ref requires ssrcorrtype 1 or 2");
    }
    if (config.ssrcorrtype != 0) {
        if (config.ssrcorrtype != 1 && config.ssrcorrtype != 2) {
            throw std::invalid_argument("ssrcorrtype expects 0, 1, or 2");
        }
        if (!config.do_orbit || config.orbtype[1] != 2) {
            throw std::invalid_argument("ssrcorrtype requires compared_orbit BRD4 or BRDM");
        }
        if (config.SSRFile.empty() && config.SSRDir.empty()) {
            throw std::invalid_argument("ssrcorrtype requires ssrfile or ssrdir");
        }
        if (!config.SSRFile.empty() && !std::filesystem::exists(config.SSRFile)) {
            throw std::invalid_argument("ssrfile does not exist: " + config.SSRFile);
        }
        if (!config.SSRDir.empty() && !std::filesystem::exists(config.SSRDir)) {
            throw std::invalid_argument("ssrdir does not exist: " + config.SSRDir);
        }
        if (config.SSRAge <= 0.0) {
            throw std::invalid_argument("ssrage must be positive");
        }
        if (config.bdsBrdClockCorr == BDSBrdClockCorr::TGD) {
            throw std::invalid_argument(
                "bds_brd_clock_corr TGD requires a BDS broadcast comparison without SSR corrections");
        }
    }
}

void Infer_Mode(Config& config) {
    // Set mode when cmp.conf has no mode line.
    if (config.modeSet) {
        Set_Mode(config);
        return;
    }

    const bool hasReferenceOrbit = !config.productFiles.sp3[0].empty() || !config.productFiles.nav[0].empty();
    const bool hasComparedOrbit = !config.productFiles.sp3[1].empty() || !config.productFiles.nav[1].empty();
    const bool hasReferenceDir = !config.productDirs[0].empty();
    const bool hasComparedDir = !config.productDirs[1].empty();
    const bool hasReferenceClock = !config.productFiles.clk[0].empty()
        || !config.productFiles.sp3[0].empty()
        || !config.productFiles.nav[0].empty();
    const bool hasComparedClock = !config.productFiles.clk[1].empty()
        || !config.productFiles.sp3[1].empty()
        || !config.productFiles.nav[1].empty();

    config.do_orbit = (hasReferenceOrbit || hasReferenceDir) && (hasComparedOrbit || hasComparedDir);
    config.do_clock = (hasReferenceClock || hasReferenceDir) && (hasComparedClock || hasComparedDir);
    if (config.do_orbit && config.do_clock) {
        config.mode = "both";
    } else if (config.do_orbit) {
        config.mode = "orb";
    } else if (config.do_clock) {
        config.mode = "clk";
    } else {
        config.mode = "both";
    }
}

void Set_Config_Value(Config& config, const std::string& key, const std::string& value) {
    // Set one config option by keyword.
    const std::string configKey = Lower_Text(key);
    if (configKey == "day") {
        if (Lower_Text(First_Word(value)) == "auto") {
            config.dayAuto = true;
            config.daySet = true;
        } else {
            std::istringstream input(value);
            input >> config.yearStart >> config.doyStart;
            if (!input) {
                throw std::invalid_argument("day expects: auto, year doy, or year doy_start doy_end");
            }
            if (!(input >> config.doyEnd)) {
                config.doyEnd = config.doyStart;
            }
            if (config.doyEnd < config.doyStart) {
                throw std::invalid_argument("day doy_end must be greater than or equal to doy_start");
            }
            config.dayAuto = false;
            config.daySet = true;
        }
    } else if (configKey == "year") {
        config.yearStart = Read_Int(value, key);
        config.dayAuto = false;
        config.daySet = true;
    } else if (configKey == "doy_st" || configKey == "doy_start") {
        config.doyStart = Read_Int(value, key);
        config.dayAuto = false;
        config.daySet = true;
    } else if (configKey == "doy_end") {
        config.doyEnd = Read_Int(value, key);
        config.dayAuto = false;
        config.daySet = true;
    } else if (configKey == "interval" || configKey == "interval(s)") {
        config.interval = Read_Double(First_Word(value), key);
    } else if (configKey == "system" || configKey == "systemused") {
        config.Sys = First_Word(value);
    } else if (configKey == "outdir") {
        config.OutDir = value;
    } else if (configKey == "mode") {
        config.mode = First_Word(value);
        config.modeSet = true;
        Set_Mode(config);
    } else if (configKey == "clkdatum") {
        config.clkdatum = Read_Clkdatum(value, key);
        config.clkdatumSet = true;
    } else if (configKey == "clkdatum_method") {
        config.clkdatumMethod = Read_Clkdatum_Method(value, key);
    } else if (configKey == "ssrcorrtype" || configKey == "ssr_corrtype") {
        config.ssrcorrtype = Read_ssrcorrtype(value, key);
    } else if (configKey == "ssrref" || configKey == "ssr_ref") {
        config.SSRRef = Read_SSRRef(value, key);
    } else if (configKey == "ssrfile" || configKey == "ssr_file") {
        config.SSRFile = value;
    } else if (configKey == "ssrdir" || configKey == "ssr_dir") {
        config.SSRDir = value;
    } else if (configKey == "ssrformat" || configKey == "ssr_format") {
        config.SSRFormat = First_Word(value);
    } else if (configKey == "ssrage" || configKey == "ssr_age") {
        config.SSRAge = Read_Double(First_Word(value), key);
    } else if (configKey == "orbtype1") {
        config.orbtype[0] = Read_Int(First_Word(value), key);
    } else if (configKey == "clktype1") {
        config.clktype[0] = Read_Int(First_Word(value), key);
    } else if (configKey == "orbtype2") {
        config.orbtype[1] = Read_Int(First_Word(value), key);
    } else if (configKey == "clktype2") {
        config.clktype[1] = Read_Int(First_Word(value), key);
    } else if (configKey == "is_cnav") {
        config.IsCNAV = Read_Bool(First_Word(value), key);
    } else if (configKey == "pco_ref" || configKey == "pco_reference") {
        throw std::invalid_argument(key + " is no longer used; SP3 orbit is treated as COM");
    } else if (configKey == "antenna_file" || configKey == "antfile") {
        config.AntFile = value;
    } else if (configKey == "brd_antfile") {
        config.BrdAntFile = value;
    } else if (configKey == "brd_pco") {
        config.brdPCO = Upper_Text(First_Word(value));
        if (config.brdPCO != "RADIAL" && config.brdPCO != "XYZ") {
            throw std::invalid_argument(key + " expects RADIAL or XYZ: " + value);
        }
    } else if (configKey == "reference_product" || configKey == "reference_type") {
        config.productTypes[0] = First_Word(value);
    } else if (configKey == "compared_product" || configKey == "compared_type") {
        config.productTypes[1] = First_Word(value);
    } else if (configKey == "reference_ac") {
        config.ACs[0] = First_Word(value);
    } else if (configKey == "compared_ac") {
        config.ACs[1] = First_Word(value);
    } else if (configKey == "reference_orbit") {
        Set_orbtype(config, 0, value, key);
    } else if (configKey == "compared_orbit") {
        Set_orbtype(config, 1, value, key);
    } else if (configKey == "reference_clock") {
        Set_clktype(config, 0, value, key);
    } else if (configKey == "compared_clock") {
        Set_clktype(config, 1, value, key);
    } else if (configKey == "reference_bdsfreq") {
        config.bdsfreq[0] = Read_BDSFreq(value, key);
    } else if (configKey == "compared_bdsfreq") {
        config.bdsfreq[1] = Read_BDSFreq(value, key);
    } else if (configKey == "bds_brd_clock_corr") {
        config.bdsBrdClockCorr = Read_BDS_Brd_Clock_Corr(value, key);
    } else if (configKey == "reference_dir") {
        Set_Product_Dir(config, 0, value);
    } else if (configKey == "compared_dir") {
        Set_Product_Dir(config, 1, value);
    } else if (configKey == "reference" || configKey == "product1") {
        Set_Product_Files(config, 0, value, key);
    } else if (configKey == "compared" || configKey == "product2") {
        Set_Product_Files(config, 1, value, key);
    } else if (configKey == "orbit") {
        config.orbtype[0] = Read_orbtype(value, key);
        config.orbtype[1] = config.orbtype[0];
    } else if (configKey == "orbit1") {
        Set_Orbit_Product(config, 0, value, key);
    } else if (configKey == "orbit2") {
        Set_Orbit_Product(config, 1, value, key);
    } else if (configKey == "clock") {
        Set_Common_Clock_Product(config, value, key);
    } else if (configKey == "clock1") {
        Set_Clock_Product(config, 0, value, key);
    } else if (configKey == "clock2") {
        Set_Clock_Product(config, 1, value, key);
    } else if (configKey == "sp3_1" || configKey == "sp3dir1" || configKey == "sp3file1") {
        config.productFiles.sp3[0] = value;
    } else if (configKey == "sp3_2" || configKey == "sp3dir2" || configKey == "sp3file2") {
        config.productFiles.sp3[1] = value;
    } else if (configKey == "sp3dir" || configKey == "sp3file") {
        config.productFiles.sp3[0] = value;
    } else if (configKey == "clk_1" || configKey == "clkdir1" || configKey == "clkfile1") {
        config.productFiles.clk[0] = value;
    } else if (configKey == "clk_2" || configKey == "clkdir2" || configKey == "clkfile2") {
        config.productFiles.clk[1] = value;
    } else if (configKey == "clkdir" || configKey == "clkfile") {
        config.productFiles.clk[0] = value;
    } else if (configKey == "nav_1" || configKey == "navfile1") {
        config.productFiles.nav[0] = value;
    } else if (configKey == "nav_2" || configKey == "navfile2") {
        config.productFiles.nav[1] = value;
    } else if (configKey == "navfile") {
        config.productFiles.nav[0] = value;
    } else if (configKey == "dcbdir") {
        config.DCBDir = value;
    } else if (configKey == "dcbtype" || configKey == "dcb_type") {
        config.DCBtype = Read_DCBtype(value, key);
    } else if (configKey == "dcb" || configKey == "dcbfile") {
        throw std::invalid_argument(key + " is no longer used; set dcbdir instead");
    } else if (configKey == "dir1" || configKey == "dir2" || configKey == "dir3") {
        throw std::invalid_argument(key + " is no longer used; set exact product file names instead");
    }
}

std::string Join_Path(const std::string& dir, const std::string& name) {
    // Join a directory and a file name.
    if (dir.empty()) {
        return name;
    }
    return (std::filesystem::path(dir) / name).generic_string();
}

std::string Format_GPSDay_String(int gpsWeek, int gpsDay) {
    // Format GPS week/day as WWWWD.
    std::ostringstream out;
    out << std::setw(4) << gpsWeek << gpsDay;
    return out.str();
}

std::string Format_Year_Doy_String(int year, int doy) {
    // Format year/DOY as YYYYDDD.
    std::ostringstream out;
    out << std::setw(7) << (year * 1000 + doy);
    return out.str();
}

std::string MakeDCBFileName(const Config& config, const ProcessDay& day) {
    // Make the daily DCB file path.
    if (config.DCBDir.empty()) {
        return "";
    }

    const std::filesystem::path DCBDir(config.DCBDir);
    const std::size_t dot = config.DCBtype.find('.');
    if (dot == std::string::npos || dot == 0 || dot + 1 >= config.DCBtype.size()) {
        throw std::invalid_argument("DCBtype expects PREFIX.EXT, for example CAS0OPSRAP.BIA");
    }
    const std::string prefix = config.DCBtype.substr(0, dot);
    const std::string extension = config.DCBtype.substr(dot + 1);
    const std::filesystem::path candidate =
        DCBDir / (prefix + "_" + day.yearDoyString + "0000_01D_01D_DCB." + extension);
    return candidate.string();
}

}  // namespace

Config Read_Config(const std::string& path) {
    // Read cmp.conf.
    Config config;
    const std::filesystem::path configPath(path);
    config.configDir = configPath.has_parent_path() ? configPath.parent_path().generic_string() : ".";

    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open config file: " + path);
    }

    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::size_t comment = line.find('!');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(line);
        if (line.empty() || line[0] == '*') {
            continue;
        }
        std::string key;
        std::string value;
        const std::size_t equal = line.find('=');
        if (equal != std::string::npos) {
            key = trim(line.substr(0, equal));
            value = trim(line.substr(equal + 1));
        } else {
            std::istringstream row(line);
            row >> key;
            std::getline(row, value);
            value = trim(value);
        }
        if (key.empty()) {
            continue;
        }
        Set_Config_Value(config, key, value);
    }

    Resolve_Config_File_Paths(config);
    Infer_Mode(config);
    Infer_Day(config);
    Check_config(config);
    return config;
}

std::string Get_SISRE_File_Base_Name(const Config& config) {
    // Return the output file base name.
    if (config.do_orbit && config.do_clock) {
        return "SISRE_";
    }
    if (config.do_orbit) {
        return "SISRE_orb_";
    }
    return "SISRE_clk_";
}

std::string Get_SISRE_FileName(const Config& config, char system, int prn) {
    // Make the output file path for one PRN.
    std::ostringstream name;
    name << Get_SISRE_File_Base_Name(config) << system << std::setw(2) << std::setfill('0') << prn << ".txt";
    return Join_Path(config.OutDir, name.str());
}

std::vector<char> Get_Process_Systems(const Config& config) {
    // Return selected systems in G/R/E/C/J order.
    constexpr std::array<char, 5> order{'G', 'R', 'E', 'C', 'J'};
    std::vector<char> result;
    for (const char system : order) {
        if (config.Sys.find(system) != std::string::npos) {
            result.push_back(system);
        }
    }
    return result;
}

int Get_SatNum(char system) {
    // Return the satellite count for one system.
    switch (system) {
        case 'G':
            return constants::GNum;
        case 'R':
            return constants::RNum;
        case 'E':
            return constants::NumE;
        case 'C':
            return constants::CNum;
        case 'J':
            return constants::JNum;
        case 'L':
            return 1;
        default:
            throw std::invalid_argument(std::string("unsupported system: ") + system);
    }
}

ProcessDay Make_Process_Day(int yearStart, int inputDoy) {
    // Make all date fields for one input DOY.
    // Roll DOY into later years when needed.
    ProcessDay day;
    day.year = yearStart;
    day.doy = inputDoy;
    if (day.doy > 365 * 3) {
        day.year += 3;
        day.doy -= 365 * 3;
    } else if (day.doy > 365 * 2) {
        day.year += 2;
        day.doy -= 365 * 2;
    } else if (day.doy > 365) {
        day.year += 1;
        day.doy -= 365;
    }

    const Gpst gpst = Day2GPST(day.year, day.doy);
    day.gpsWeek = gpst.week;
    day.gpsDay = gpst.day;
    day.gpsDayString = Format_GPSDay_String(day.gpsWeek, day.gpsDay);
    day.yearDoyString = Format_Year_Doy_String(day.year, day.doy);
    return day;
}

void Set_Found_Product_File(ProductFileNames& files, int product, const std::string& fileName) {
    // Store a product file by extension.
    const std::size_t index = static_cast<std::size_t>(product);
    const std::string lowerName = Lower_Text(fileName);
    if (Has_Ending(lowerName, ".sp3")) {
        files.sp3[index] = fileName;
    } else if (Has_Ending(lowerName, ".clk")) {
        files.clk[index] = fileName;
    } else {
        files.nav[index] = fileName;
    }
}

bool File_Exists(const std::filesystem::path& path) {
    // Check that the path is a file.
    return std::filesystem::exists(path) && std::filesystem::is_regular_file(path);
}

std::string Two_Digit_Year(int year) {
    // Format year as YY.
    std::ostringstream out;
    out << std::setw(2) << std::setfill('0') << (year % 100);
    return out.str();
}

std::string Three_Digit_Doy(int doy) {
    // Format DOY as DDD.
    std::ostringstream out;
    out << std::setw(3) << std::setfill('0') << doy;
    return out.str();
}

std::string Replace_All(std::string text, const std::string& from, const std::string& to) {
    // Replace all occurrences of one token.
    if (from.empty()) {
        return text;
    }
    std::size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
    return text;
}

std::string Expand_SSR_Date_Format(std::string format, const ProcessDay& day) {
    // Expand SSR date tokens for one process day.
    const std::string yyyy = std::to_string(day.year);
    const std::string yy = Two_Digit_Year(day.year);
    const std::string ddd = Three_Digit_Doy(day.doy);
    format = Upper_Text(format);
    format = Replace_All(format, "GPSDAY", day.gpsDayString);
    format = Replace_All(format, "YYYYDDD", day.yearDoyString);
    format = Replace_All(format, "DDDYY", ddd + yy);
    format = Replace_All(format, "YYDDD", yy + ddd);
    format = Replace_All(format, "YYYY", yyyy);
    format = Replace_All(format, "DDD", ddd);
    format = Replace_All(format, "YY", yy);
    format = Replace_All(format, "GPSW", std::to_string(day.gpsWeek));
    format = Replace_All(format, "GPSD", std::to_string(day.gpsDay));
    return format;
}

std::string Make_SSR_FileName(const Config& config, const ProcessDay& day) {
    // Return the SSR file for one day.
    if (!config.SSRFile.empty()) {
        return config.SSRFile;
    }
    if (config.SSRDir.empty()) {
        return "";
    }

    const std::filesystem::path root(config.SSRDir);
    const std::string dateText = Expand_SSR_Date_Format(config.SSRFormat, day);
    const std::filesystem::path exactPath = root / dateText;
    if (File_Exists(exactPath)) {
        return exactPath.string();
    }

    const std::string lowerDateText = Lower_Text(dateText);
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string lowerName = Lower_Text(entry.path().filename().string());
        if (lowerName.find(lowerDateText) != std::string::npos) {
            return entry.path().string();
        }
    }
    throw std::invalid_argument("SSR file not found in " + config.SSRDir
                                + " with date format " + config.SSRFormat
                                + " for " + day.yearDoyString);
}

void Find_Short_Product_Files(ProductFileNames& files, const Config& config, int product, const std::filesystem::path& root,
                            const ProcessDay& day) {
    // Search short product names in one directory.
    const std::size_t index = static_cast<std::size_t>(product);
    const std::string ac = Lower_Text(config.ACs[index]);
    // Require AC prefix for precise short names.
    if (!ac.empty()) {
        const std::string shortNameBase = ac + day.gpsDayString;
        const std::filesystem::path sp3 = root / (shortNameBase + ".sp3");
        if (files.sp3[index].empty()
            && (config.orbtypeText[index] == "SP3" || config.clktypeText[index] == "SP3")
            && File_Exists(sp3)) {
            files.sp3[index] = sp3.string();
        }

        const std::filesystem::path clk = root / (shortNameBase + ".clk");
        if (files.clk[index].empty() && config.clktypeText[index] == "CLK" && File_Exists(clk)) {
            files.clk[index] = clk.string();
        }
    }

    const std::string yy = Two_Digit_Year(day.year);
    const std::string ddd = Three_Digit_Doy(day.doy);
    std::vector<std::string> brdNames;
    if (config.orbtypeText[index] == "BRD4" || config.clktypeText[index] == "BRD4") {
        brdNames.push_back("brd4" + ddd + "0." + yy + "p");
    }
    if (config.orbtypeText[index] == "BRDM" || config.clktypeText[index] == "BRDM") {
        brdNames.push_back("brdm" + ddd + "0." + yy + "p");
    }
    for (const std::string& brdName : brdNames) {
        const std::filesystem::path brd = root / brdName;
        if (files.nav[index].empty() && File_Exists(brd)) {
            files.nav[index] = brd.string();
        }
    }
}

bool Is_SP3_FileName(const std::string& lowerName) {
    // Check long SP3 name.
    return Has_Ending(lowerName, "_orb.sp3");
}

bool Is_CLK_FileName(const std::string& lowerName) {
    // Check long CLK name.
    return Has_Ending(lowerName, "_clk.clk");
}

bool Is_Brd_FileName(const std::string& lowerName, const std::string& brdtype) {
    // Check BRD4/BRDM file prefix.
    const std::string text = Lower_Text(brdtype);
    if (text == "brd4") {
        return lowerName.rfind("brd4", 0) == 0;
    }
    if (text == "brdm") {
        return lowerName.rfind("brdm", 0) == 0;
    }
    return false;
}

bool Match_Product_AC(const Config& config, int product, const std::string& lowerName) {
    // Check AC file-name prefix.
    const std::string ac = Lower_Text(config.ACs[static_cast<std::size_t>(product)]);
    return ac.empty() || lowerName.rfind(ac, 0) == 0;
}

void Set_Long_Product_Candidate(ProductFileNames& files, const Config& config, int product, const std::filesystem::path& path) {
    // Store a matching long-name product file.
    const std::size_t index = static_cast<std::size_t>(product);
    const std::string lowerName = Lower_Text(path.filename().string());
    const std::string orbtypeName = config.orbtypeText[index];
    const std::string clktypeName = config.clktypeText[index];
    if (files.sp3[index].empty()
        && Is_SP3_FileName(lowerName)
        && (orbtypeName == "SP3" || clktypeName == "SP3")) {
        files.sp3[index] = path.string();
    } else if (files.clk[index].empty() && Is_CLK_FileName(lowerName) && clktypeName == "CLK") {
        files.clk[index] = path.string();
    } else if (files.nav[index].empty()
        && ((orbtypeName == "BRD4" || orbtypeName == "BRDM") && Is_Brd_FileName(lowerName, orbtypeName)
            || (clktypeName == "BRD4" || clktypeName == "BRDM") && Is_Brd_FileName(lowerName, clktypeName))) {
        files.nav[index] = path.string();
    }
}

void Find_Long_Product_Files(ProductFileNames& files, const Config& config, int product, const std::filesystem::path& root,
                          const ProcessDay& day) {
    // Search long product names in one directory.
    const std::size_t index = static_cast<std::size_t>(product);
    const std::string ac = Lower_Text(config.ACs[index]);
    if (ac.empty()) {
        return;
    }
    // Match YYYYDDD and AC prefix.
    const std::string dateKey = Lower_Text(day.yearDoyString);
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string lowerName = Lower_Text(entry.path().filename().string());
        if (lowerName.find(dateKey) == std::string::npos) {
            continue;
        }
        if (!Match_Product_AC(config, product, lowerName)) {
            continue;
        }
        Set_Long_Product_Candidate(files, config, product, entry.path());
    }
}

void Find_Product_Files_In_Dir(ProductFileNames& files, const Config& config, int product, const std::string& dir,
                           const ProcessDay& day) {
    // Search one product directory.
    if (dir.empty()) {
        return;
    }
    const std::filesystem::path root(dir);
    if (!std::filesystem::exists(root)) {
        throw std::invalid_argument("product directory does not exist: " + dir);
    }
    Find_Long_Product_Files(files, config, product, root, day);
    Find_Short_Product_Files(files, config, product, root, day);
}

ProductFileNames Make_Product_FileNames(const Config& config, const ProcessDay& day) {
    // Make product file paths for one day.
    ProductFileNames files = config.productFiles;
    Find_Product_Files_In_Dir(files, config, 0, config.productDirs[0], day);
    Find_Product_Files_In_Dir(files, config, 1, config.productDirs[1], day);
    files.dcb = MakeDCBFileName(config, day);
    return files;
}

struct ProductRuntime {
    // Files and data for one product.
    std::ifstream sp3Input;
    std::ifstream clkInput;
    std::string sp3PendingLine;
    std::string clkPendingLine;
    Sp3Head sp3Head;
    NavHead navHead;
    Sp3Data sp3Data;
    ClockData clkData;
    std::vector<std::vector<NavRecord>> navRecords;
    std::vector<std::vector<GlonassNavRecord>> glonassRecords;
    MGEXDCB dcb;
};

struct Saved_Brd_NavData {
    // Broadcast records saved from the previous day.
    std::vector<std::vector<NavRecord>> navRecords;
    std::vector<std::vector<GlonassNavRecord>> glonassRecords;
};

struct OutputFiles {
    // Output files indexed by PRN.
    std::vector<std::ofstream> files;
};

struct ClkDatumState {
    // State for CMED/CAVG datum (from initial clock bias).
    std::vector<double> PredClk;
    double Bias = 0.0;
};

struct SISREValues {
    // SISRE values for one output row.
    double sisre = 0.0;
    double orbit = 0.0;
};

int Get_orbtype_i(const Config& config, int product) {
    // Return the orbit type for one product.
    return product == 0 ? config.orbtype[0] : config.orbtype[1];
}

int Get_clktype_i(const Config& config, int product) {
    // Return the clock type for one product.
    return product == 0 ? config.clktype[0] : config.clktype[1];
}

bool Product_Uses_Brd(const Config& config, int product) {
    // Check whether this product reads broadcast navigation.
    return (config.do_orbit && Get_orbtype_i(config, product) == 2)
        || (config.do_clock && Get_clktype_i(config, product) == 0);
}

bool Needs_BDS_Clock_Frequency_Alignment(const Config& config, char system) {
    // Check whether the compared BDS clock must be aligned to the reference frequency.
    return config.do_clock
        && system == 'C'
        && !Product_Uses_Brd(config, 0)
        && config.bdsfreq[0] != config.bdsfreq[1];
}

bool Uses_BDS_Broadcast_TGD(const Config& config) {
    // Use TGD from the selected BDS broadcast record without SSR corrections.
    return config.bdsBrdClockCorr == BDSBrdClockCorr::TGD && config.ssrcorrtype == 0;
}

BDSFreq BDSFreq_For_Brd_Product(const Config& config, int product) {
    // Keep BDS broadcast clock at B3.
    (void)config;
    (void)product;
    return BDSFreq::B3;
}

bool Has_SSR_Correction(const SSRData* ssrData, int PRNIndex) {
    // Check whether this satellite has SSR orbit or clock correction.
    if (ssrData == nullptr || PRNIndex < 0 || PRNIndex >= constants::TotalSatNum) {
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(PRNIndex);
    const Vector3& dOrb = ssrData->dOrb[index];
    return ssrData->iode[index] >= 0.0
        || std::abs(ssrData->dClk[index]) > 1.0e-12
        || std::abs(dOrb[0]) > 1.0e-12
        || std::abs(dOrb[1]) > 1.0e-12
        || std::abs(dOrb[2]) > 1.0e-12;
}

bool Has_SSR_Epoch(const SSRData* ssrData, int obsWeek, double obsSec, double ssrAge) {
    // Check whether SSR data covers the current epoch.
    if (ssrData == nullptr || ssrData->week == 0) {
        return false;
    }
    const double dt = (ssrData->week - obsWeek) * 604800.0 + ssrData->sow - obsSec;
    return std::abs(dt) <= ssrAge;
}

bool Is_Invalid_Position(const Vector3& position) {
    // Check the missing position marker.
    return std::abs(position[0] - 9999.0) < 0.1
        && std::abs(position[1] - 9999.0) < 0.1
        && std::abs(position[2] - 9999.0) < 0.1;
}

bool Is_Invalid_Clock(double clock) {
    // Check the missing clock marker.
    return std::abs(clock - 9999.0) < 0.1;
}

bool Has_DCB_Value(double value) {
    // Check whether a DCB/BIA value is present.
    return std::abs(value - 99.0) > 1.0e-12;
}

double BDS_Clock_Combination_Offset(const std::array<double, 6>& dcb, BDSFreq freq, bool& valid) {
    // Return the BDS clock-combination offset relative to B3.
    if (freq == BDSFreq::B3) {
        valid = true;
        return 0.0;
    }
    if (freq == BDSFreq::B1B2) {
        valid = Has_DCB_Value(dcb[3]) && Has_DCB_Value(dcb[4]);
        return valid ? 2.2606 * dcb[3] - 1.2606 * dcb[4] : 0.0;
    }
    valid = Has_DCB_Value(dcb[0]);
    return valid ? 2.9437 * dcb[0] : 0.0;
}

double BDS_Clock_Frequency_Correction(const Config& config,
                                      const ProductRuntime& compared,
                                      char system, int prn, bool& valid) {
    // Convert the compared BDS clock from its native frequency to the reference frequency.
    valid = true;
    if (!Needs_BDS_Clock_Frequency_Alignment(config, system)) {
        return 0.0;
    }
    if (Uses_BDS_Broadcast_TGD(config) && Product_Uses_Brd(config, 1)) {
        return 0.0;
    }
    if (prn < 1 || prn > constants::CNum) {
        valid = false;
        return 0.0;
    }

    const auto& dcb = compared.dcb.bds[static_cast<std::size_t>(prn)];
    bool comparedValid = false;
    bool referenceValid = false;
    const double comparedOffset = BDS_Clock_Combination_Offset(dcb, config.bdsfreq[1], comparedValid);
    const double referenceOffset = BDS_Clock_Combination_Offset(dcb, config.bdsfreq[0], referenceValid);
    valid = comparedValid && referenceValid;
    return valid ? comparedOffset - referenceOffset : 0.0;
}

bool Has_Huge_Component(const Vector3& value, double limit) {
    // Check the coordinate difference limit.
    return std::abs(value[0]) > limit || std::abs(value[1]) > limit || std::abs(value[2]) > limit;
}

double Delta_GPS_Seconds(int weekA, double secA, int weekB, double secB) {
    // Return GPS time difference in seconds.
    return (weekA - weekB) * 604800.0 + secA - secB;
}

void Require_Existing_File(const std::string& path, const std::string& label) {
    // Check that a required file exists.
    if (path.empty()) {
        throw std::runtime_error("missing " + label);
    }
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error(label + " does not exist: " + path);
    }
}

void Read_SP3_File(ProductRuntime& runtime, const std::string& path) {
    // Open SP3 and read the first data window.
    Require_Existing_File(path, "SP3 file");
    runtime.sp3Input.open(path);
    if (!runtime.sp3Input) {
        throw std::runtime_error("failed to open SP3 file: " + path);
    }
    runtime.sp3Head = ReadSP3Head(runtime.sp3Input, &runtime.sp3PendingLine);
    if (ReadSP3Data(runtime.sp3Input, runtime.sp3Head, runtime.sp3Data, constants::Sp3EpochSlots,
                   &runtime.sp3PendingLine)
        <= 0) {
        throw std::runtime_error("SP3 file has no usable epochs: " + path);
    }
}

void Read_CLK_File(ProductRuntime& runtime, const std::string& path) {
    // Open CLK and read the first epoch.
    Require_Existing_File(path, "CLK file");
    runtime.clkInput.open(path);
    if (!runtime.clkInput) {
        throw std::runtime_error("failed to open CLK file: " + path);
    }
    ReadClkHead(runtime.clkInput);
    if (ReadClkData(runtime.clkInput, runtime.clkData, 1, &runtime.clkPendingLine) <= 0) {
        throw std::runtime_error("CLK file has no usable epochs: " + path);
    }
}

RuntimeOptions Runtime_Options_From_Config(const Config& config) {
    // Make navigation options from config.
    RuntimeOptions options;
    options.IsCNAV = config.IsCNAV;
    return options;
}

void Read_Nav_File(ProductRuntime& runtime, const Config& config, const std::string& path, char system, int satNum) {
    // Read broadcast navigation for one system.
    Require_Existing_File(path, "navigation file");
    const NavHead head = ReadNavHeadFile(path);
    runtime.navHead = head;
    const RuntimeOptions options = Runtime_Options_From_Config(config);

    if (system == 'R') {
        runtime.glonassRecords.assign(static_cast<std::size_t>(satNum + 1), {});
        for (const GlonassNavRecordEntry& entry : ReadNavData_RFile(path, head)) {
            if (entry.prn >= 1 && entry.prn <= satNum) {
                runtime.glonassRecords[static_cast<std::size_t>(entry.prn)].push_back(entry.record);
            }
        }
        return;
    }

    runtime.navRecords.assign(static_cast<std::size_t>(satNum + 1), {});
    for (const NavRecordEntry& entry : ReadNavDataFile(path, system, head, options)) {
        if (entry.prn >= 1 && entry.prn <= satNum) {
            runtime.navRecords[static_cast<std::size_t>(entry.prn)].push_back(entry.record);
        }
    }
}

void Read_Product_Data(ProductRuntime& runtime, const Config& config, const ProductFileNames& files,
                        int product, char system, int satNum) {
    // Read selected product files.
    const std::size_t index = static_cast<std::size_t>(product);
    const int orbtypeValue = Get_orbtype_i(config, product);
    const int clktypeValue = Get_clktype_i(config, product);

    if ((config.do_orbit && orbtypeValue == 1) || (config.do_clock && clktypeValue == 2)) {
        Read_SP3_File(runtime, files.sp3[index]);
    }
    if (config.do_clock && clktypeValue == 1) {
        Read_CLK_File(runtime, files.clk[index]);
    }
    if ((config.do_orbit && orbtypeValue == 2) || (config.do_clock && clktypeValue == 0)) {
        Read_Nav_File(runtime, config, files.nav[index], system, satNum);
        if (system == 'R' || (system == 'C' && !Uses_BDS_Broadcast_TGD(config))) {
            Require_Existing_File(files.dcb, "DCB file");
            runtime.dcb = Read_MGEXDCB(files.dcb);
        }
    }
    if (product == 1 && Needs_BDS_Clock_Frequency_Alignment(config, system)
        && !(Uses_BDS_Broadcast_TGD(config) && Product_Uses_Brd(config, 1))) {
        Require_Existing_File(files.dcb, "DCB file");
        runtime.dcb = Read_MGEXDCB(files.dcb);
    }
}

void Append_Saved_Brd_NavData(ProductRuntime& runtime, const Saved_Brd_NavData& NavData_Save,
                              char system, int satNum) {
    // Append previous-day broadcast records after current-day records.
    if (system == 'R') {
        if (NavData_Save.glonassRecords.empty()) {
            return;
        }
        if (runtime.glonassRecords.size() < static_cast<std::size_t>(satNum + 1)) {
            runtime.glonassRecords.resize(static_cast<std::size_t>(satNum + 1));
        }
        for (int prn = 1; prn <= satNum && prn < static_cast<int>(NavData_Save.glonassRecords.size()); ++prn) {
            auto& records = runtime.glonassRecords[static_cast<std::size_t>(prn)];
            const auto& olderRecords = NavData_Save.glonassRecords[static_cast<std::size_t>(prn)];
            records.insert(records.end(), olderRecords.begin(), olderRecords.end());
        }
        return;
    }

    if (NavData_Save.navRecords.empty()) {
        return;
    }
    if (runtime.navRecords.size() < static_cast<std::size_t>(satNum + 1)) {
        runtime.navRecords.resize(static_cast<std::size_t>(satNum + 1));
    }
    for (int prn = 1; prn <= satNum && prn < static_cast<int>(NavData_Save.navRecords.size()); ++prn) {
        auto& records = runtime.navRecords[static_cast<std::size_t>(prn)];
        const auto& olderRecords = NavData_Save.navRecords[static_cast<std::size_t>(prn)];
        records.insert(records.end(), olderRecords.begin(), olderRecords.end());
    }
}

void Save_Brd_NavData(Saved_Brd_NavData& NavData_Save, const ProductRuntime& runtime, char system) {
    // Save broadcast records for the next day.
    if (system == 'R') {
        NavData_Save.glonassRecords = runtime.glonassRecords;
        return;
    }
    NavData_Save.navRecords = runtime.navRecords;
}

void Read_Next_SP3(ProductRuntime& runtime, int obsWeek, double obsSec) {
    // Move the SP3 window to the current epoch.
    while (Delta_GPS_Seconds(obsWeek, obsSec, runtime.sp3Data.gpsWeek[5], runtime.sp3Data.gpsSec[5]) >= 0.0) {
        if (ReadSP3Data(runtime.sp3Input, runtime.sp3Head, runtime.sp3Data, 1, &runtime.sp3PendingLine) <= 0) {
            break;
        }
    }
}

void Read_Next_CLK(ProductRuntime& runtime, int obsWeek, double obsSec) {
    // Move CLK data to the current epoch.
    while (Delta_GPS_Seconds(obsWeek, obsSec, runtime.clkData.gpsWeek[1], runtime.clkData.gpsSec[1]) >= 0.0) {
        if (ReadClkData(runtime.clkInput, runtime.clkData, 1, &runtime.clkPendingLine) <= 0) {
            break;
        }
    }
}

void Read_Next_Product_Data(ProductRuntime& runtime, const Config& config, int product, int obsWeek, double obsSec) {
    // Move product buffers to the current epoch.
    const int orbtypeValue = Get_orbtype_i(config, product);
    const int clktypeValue = Get_clktype_i(config, product);
    if ((config.do_orbit && orbtypeValue == 1) || (config.do_clock && clktypeValue == 2)) {
        Read_Next_SP3(runtime, obsWeek, obsSec);
    }
    if (config.do_clock && clktypeValue == 1) {
        Read_Next_CLK(runtime, obsWeek, obsSec);
    }
}

SatState Cal_Sat_Pos_brd(const ProductRuntime& runtime, const Config& config, int product, char system, int obsWeek,
                         double obsSec, int prn, const SSRData* ssrData) {
    // Calculate satellite state from broadcast navigation.
    CalSatPosOptions options;
    options.runtime = Runtime_Options_From_Config(config);
    options.bdsfreq = BDSFreq_For_Brd_Product(config, product);
    if (system == 'C' && Uses_BDS_Broadcast_TGD(config)) {
        options.bdsfreq = config.bdsfreq[0];
        options.bdsBrdClockB3 = false;
        options.useBroadcastTGD = true;
    }
    options.bdsSSRIsB2b = system == 'C' && ssrData != nullptr && !ssrData->isBNC;
    if (ssrData != nullptr && product == 1) {
        const int PRNIndex = Get_PRNIndex(system, prn);
        if (Has_SSR_Correction(ssrData, PRNIndex)) {
            options.useSSRNavIode = true;
            options.ssrIode = ssrData->iode[static_cast<std::size_t>(PRNIndex)];
        }
    }
    if (system == 'C' && prn >= 1 && prn <= constants::CNum) {
        options.dcb = runtime.dcb.bds[static_cast<std::size_t>(prn)];
    } else if (system == 'R' && prn >= 1 && prn <= constants::RNum) {
        const auto& dcb = runtime.dcb.glonass[static_cast<std::size_t>(prn)];
        options.dcb[0] = dcb[0];
        options.dcb[1] = dcb[1];
    }
    if (system == 'R') {
        if (prn >= static_cast<int>(runtime.glonassRecords.size())) {
            return SatState{{9999.0, 9999.0, 9999.0}, {9999.0, 9999.0, 9999.0}, 9999.0, 0.0};
        }
        return Cal_Sat_Pos_g(obsWeek, obsSec - runtime.navHead.leapSeconds, prn,
                          runtime.glonassRecords[static_cast<std::size_t>(prn)], options);
    }
    if (prn >= static_cast<int>(runtime.navRecords.size())) {
        return SatState{{9999.0, 9999.0, 9999.0}, {9999.0, 9999.0, 9999.0}, 9999.0, 0.0};
    }
    double navSec = obsSec;
    if (system == 'C') {
        navSec -= 14.0;
    }
    return Cal_Sat_Pos_nav(system, obsWeek, navSec, prn, runtime.navRecords[static_cast<std::size_t>(prn)], options);
}

SatState Get_Sat_State(const ProductRuntime& runtime, const Config& config, int product,
                       char system, int prn, int obsWeek, double obsSec, const SSRData* ssrData) {
    // Get satellite state for one product.
    const int PRNIndex = Get_PRNIndex(system, prn);
    if (PRNIndex < 0) {
        return SatState{{9999.0, 9999.0, 9999.0}, {9999.0, 9999.0, 9999.0}, 9999.0, 0.0};
    }

    const int orbtype_i = Get_orbtype_i(config, product);
    const int clktype_i = Get_clktype_i(config, product);
    SatState Sat;
    bool hasSat = false;

    if (config.do_orbit) {
        if (orbtype_i == 1) {
            Sat = Cal_Sat_Pos_sp3(obsWeek, obsSec + runtime.sp3Head.sp3Time, PRNIndex, runtime.sp3Data);
        } else {
            Sat = Cal_Sat_Pos_brd(runtime, config, product, system, obsWeek, obsSec, prn, ssrData);
        }
        hasSat = true;
    }

    if (config.do_clock) {
        if (clktype_i == 1) {
            if (!hasSat && orbtype_i == 1) {
                Sat = Cal_Sat_Pos_sp3(obsWeek, obsSec + runtime.sp3Head.sp3Time, PRNIndex, runtime.sp3Data);
                hasSat = true;
            }
            Sat.clock = Cal_Sat_Clk(obsWeek, obsSec + runtime.sp3Head.sp3Time, PRNIndex, runtime.clkData);
        } else if (clktype_i == 2) {
            if (!hasSat || orbtype_i != 1) {
                Sat = Cal_Sat_Pos_sp3(obsWeek, obsSec + runtime.sp3Head.sp3Time, PRNIndex, runtime.sp3Data);
                hasSat = true;
            }
        } else {
            if (!hasSat || orbtype_i != 2) {
                Sat = Cal_Sat_Pos_brd(runtime, config, product, system, obsWeek, obsSec, prn, ssrData);
                hasSat = true;
            }
        }
    }

    return Sat;
}

double Median_Clk_Datum(const std::vector<double>& values, int satNum, bool includeZero = false) {
    // Calculate median datum (from initial clock bias).
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(satNum));
    for (int i = 1; i <= satNum; ++i) {
        if (includeZero || values[static_cast<std::size_t>(i)] != 0.0) {
            samples.push_back(values[static_cast<std::size_t>(i)]);
        }
    }
    if (samples.empty()) {
        return 0.0;
    }

    // Keep the upper median for an even number of samples.
    const auto middle = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), middle, samples.end());
    return *middle;
}

double Average_Clk_Datum(const std::vector<double>& values, int satNum, bool includeZero = false) {
    // Calculate average datum (from initial clock bias).
    double sum = 0.0;
    int count = 0;
    for (int i = 1; i <= satNum; ++i) {
        if (!includeZero && values[static_cast<std::size_t>(i)] == 0.0) {
            continue;
        }
        sum += values[static_cast<std::size_t>(i)];
        ++count;
    }
    if (count >= 1) {
        return sum / count;
    }
    return 99.0;
}

void Ensure_Clk_Datum_State_Size(ClkDatumState& state, int satNum) {
    // Resize datum state (from initial clock bias).
    const std::size_t requiredSize = static_cast<std::size_t>(satNum + 1);
    if (state.PredClk.size() != requiredSize) {
        state.PredClk.assign(requiredSize, 0.0);
        state.Bias = 0.0;
    }
}

double Continuous_Median_Clk_Datum(const std::vector<double>& values, int satNum, ClkDatumState& state) {
    // Calculate continuous median datum (from initial clock bias).
    Ensure_Clk_Datum_State_Size(state, satNum);

    std::vector<double> diffValues(static_cast<std::size_t>(satNum + 1), 0.0);
    int diffCount = 0;
    bool satelliteChanged = false;
    for (int i = 1; i <= satNum; ++i) {
        const double value = values[static_cast<std::size_t>(i)];
        const double previous = state.PredClk[static_cast<std::size_t>(i)];
        if (value != 0.0 && previous != 0.0) {
            ++diffCount;
            diffValues[static_cast<std::size_t>(diffCount)] = value - previous;
        } else if (value == 0.0 && previous == 0.0) {
            continue;
        } else {
            satelliteChanged = true;
        }
    }

    if (satelliteChanged && diffCount > 0) {
        const double med = Median_Clk_Datum(values, satNum);
        double previousMed = Median_Clk_Datum(state.PredClk, satNum);
        // All packed increments are valid, including zero changes.
        const double diff = Median_Clk_Datum(diffValues, diffCount, true);
        previousMed = previousMed - state.Bias + diff;
        state.Bias = med - previousMed;
        return previousMed;
    }
    return Median_Clk_Datum(values, satNum) - state.Bias;
}

double Continuous_Average_Clk_Datum(const std::vector<double>& values, int satNum, ClkDatumState& state) {
    // Calculate continuous average datum (from initial clock bias).
    Ensure_Clk_Datum_State_Size(state, satNum);

    std::vector<double> diffValues(static_cast<std::size_t>(satNum + 1), 0.0);
    int diffCount = 0;
    bool satelliteChanged = false;
    for (int i = 1; i <= satNum; ++i) {
        const double value = values[static_cast<std::size_t>(i)];
        const double previous = state.PredClk[static_cast<std::size_t>(i)];
        if (value != 0.0 && previous != 0.0) {
            ++diffCount;
            diffValues[static_cast<std::size_t>(diffCount)] = value - previous;
        } else if (value == 0.0 && previous == 0.0) {
            continue;
        } else {
            satelliteChanged = true;
        }
    }

    if (satelliteChanged && diffCount > 0) {
        const double med = Average_Clk_Datum(values, satNum);
        double previousMed = Average_Clk_Datum(state.PredClk, satNum);
        // All packed increments are valid, including zero changes.
        const double diff = Average_Clk_Datum(diffValues, diffCount, true);
        previousMed = previousMed - state.Bias + diff;
        state.Bias = med - previousMed;
        return previousMed;
    }

    // Use the current average while keeping the accumulated datum bias.
    return Average_Clk_Datum(values, satNum) - state.Bias;
}

bool Use_RminusClk_Datum(const Config& config) {
    // Use R-C only when orbit and clock are enabled.
    if (!config.do_clock) {
        return false;
    }
    if (!config.do_orbit) {
        return false;
    }
    if (config.clkdatum == "CLK") {
        return false;
    }
    return true;
}

std::string Get_Clkdatum(const Config& config) {
    // Return clkdatum (from initial clock bias).
    if (!config.do_clock) {
        return "";
    }
    if (!config.do_orbit) {
        return "CLK";
    }
    return config.clkdatum;
}

double Cal_Clk_Datum(const Config& config, const std::vector<double>& values,
                     int satNum, ClkDatumState& state) {
    // Calculate datum by config.clkdatumMethod (from initial clock bias).
    if (config.clkdatumMethod == "CMED") {
        return Continuous_Median_Clk_Datum(values, satNum, state);
    }
    if (config.clkdatumMethod == "CAVG") {
        return Continuous_Average_Clk_Datum(values, satNum, state);
    }
    Ensure_Clk_Datum_State_Size(state, satNum);
    return Median_Clk_Datum(values, satNum);
}

bool Contains_Text(const std::string& text, const std::string& needle) {
    // Check whether text contains a substring.
    return text.find(needle) != std::string::npos;
}

const Antenna* Get_Ant_Pointer(const std::array<Antenna, constants::TotalSatNum>& antennas, char system, int prn) {
    // Get antenna data by system and PRN.
    const int index = Get_PRNIndex(system, prn);
    if (index < 0) {
        return nullptr;
    }
    const Antenna& antenna = antennas[static_cast<std::size_t>(index)];
    if (antenna.pco.frequencyCount() == 0) {
        return nullptr;
    }
    return &antenna;
}

const Antenna* Get_Ant_By_Index(const std::array<Antenna, constants::TotalSatNum>& antennas,
                                int oneBasedIndex) {
    // Get antenna data by one-based index.
    if (oneBasedIndex < 1 || oneBasedIndex > constants::TotalSatNum) {
        return nullptr;
    }
    const Antenna& antenna = antennas[static_cast<std::size_t>(oneBasedIndex - 1)];
    if (antenna.pco.frequencyCount() == 0) {
        return nullptr;
    }
    return &antenna;
}

double PCO_Up(const Antenna* antenna, int Get_Frequency_Index) {
    // Return the PCO up component.
    if (antenna == nullptr || Get_Frequency_Index < 0 || Get_Frequency_Index >= antenna->pco.frequencyCount()) {
        return 0.0;
    }
    return antenna->pco.at(Get_Frequency_Index)[2];
}

int PCO_Index(const Antenna* antenna, const std::string& frequency) {
    // Return the PCO index for a frequency name.
    if (antenna == nullptr) {
        return -1;
    }
    for (std::size_t index = 0; index < antenna->frequency.size(); ++index) {
        if (antenna->frequency[index] == frequency) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

double PCO_Up(const Antenna* antenna, const std::string& frequency) {
    // Return the PCO up component for a frequency name.
    return PCO_Up(antenna, PCO_Index(antenna, frequency));
}

double BDS_Broadcast_Effective_PCO(const Antenna* antenna, int yearDoy) {
    // Keep the legacy antenna rule for SSR and pre-2024 processing.
    if (yearDoy >= BDS_PRN_Renumbering_YearDoy
        && antenna != nullptr
        && Contains_Text(antenna->antennaType, "BEIDOU-2I")
        && (antenna->svn == 17 || antenna->svn == 19)) {
        return BDS2_IPCO;
    }
    return PCO_Up(antenna, "C06");
}

double BDS_Broadcast_Clock_PCO(const Antenna* antenna, BDSFreq referenceFreq, int yearDoy) {
    // Return reference clock PCO minus the effective broadcast PCO.
    const double b3PCO = PCO_Up(antenna, "C06");
    const double broadcastPCO = BDS_Broadcast_Effective_PCO(antenna, yearDoy);
    if (referenceFreq == BDSFreq::B3) {
        return b3PCO - broadcastPCO;
    }
    if (referenceFreq == BDSFreq::B1B2) {
        const double precisePCO = 2.2606 * PCO_Up(antenna, "C01")
            - 1.2606 * PCO_Up(antenna, "C05");
        return precisePCO - broadcastPCO;
    }
    const double precisePCO = 2.94 * PCO_Up(antenna, "C02")
        - 1.94 * b3PCO;
    return precisePCO - broadcastPCO;
}

double BDS_Current_Brd_Radial_PCO(const Antenna* antenna, int yearDoy) {
    // Return the effective broadcast PCO used as the radial correction.
    return BDS_Broadcast_Effective_PCO(antenna, yearDoy);
}

double BDS_PCO_Up(const Antenna* antenna, const std::string& frequency,
                  const std::string& fileOption, int prn) {
    // Missing calibration is not a zero PCO.
    const int index = PCO_Index(antenna, frequency);
    if (antenna == nullptr || index < 0 || index >= antenna->pco.frequencyCount()) {
        throw std::runtime_error(fileOption + ": missing valid " + frequency
                                 + " PCO for BDS PRN " + std::to_string(prn));
    }
    const double pco = antenna->pco.at(index)[2];
    if (!std::isfinite(pco)) {
        throw std::runtime_error(fileOption + ": invalid " + frequency
                                 + " PCO for BDS PRN " + std::to_string(prn));
    }
    return pco;
}

double BDS_Reference_Clock_PCO(const Antenna* antenna, BDSFreq frequency, int prn) {
    // Use the antenna model of the precise clock product, not the broadcast model.
    if (frequency == BDSFreq::B3) {
        return BDS_PCO_Up(antenna, "C06", "antfile", prn);
    }
    if (frequency == BDSFreq::B1B2) {
        return 2.2606 * BDS_PCO_Up(antenna, "C01", "antfile", prn)
            - 1.2606 * BDS_PCO_Up(antenna, "C05", "antfile", prn);
    }
    return 2.94 * BDS_PCO_Up(antenna, "C02", "antfile", prn)
        - 1.94 * BDS_PCO_Up(antenna, "C06", "antfile", prn);
}

double BDS_Brd_PCO(const Antenna* reference, const Antenna* broadcast, int prn) {
    // Both calibrations must describe the same physical satellite on this day.
    if (reference == nullptr || broadcast == nullptr || reference->svn <= 0
        || broadcast->svn <= 0 || reference->svn != broadcast->svn) {
        throw std::runtime_error("BDS PRN " + std::to_string(prn)
            + ": antfile/brd_antfile SVN mismatch or missing valid block ("
            + std::to_string(reference == nullptr ? 0 : reference->svn) + "/"
            + std::to_string(broadcast == nullptr ? 0 : broadcast->svn) + ")");
    }
    return BDS_PCO_Up(broadcast, "C06", "brd_antfile", prn);
}

double BDS_Old_Brd_Radial_PCO(int prn) {
    // Use the old BDS broadcast radial PCO values before 2024.
    if (prn == 13) return 1.27;
    if (prn <= 15) return 1.18;
    if (prn == 16) return 1.27;
    if (prn >= 17 && prn <= 24) return 1.22;
    if (prn >= 25 && prn <= 30) return 1.09;
    if (prn >= 34 && prn <= 37) return 1.07;
    return 1.18;
}

bool BDS_PRN_Switches_On_Day(const BDSSwitchDayPRNs& switchDays, const ProcessDay& day, int prn) {
    // Skip the BDS satellite on the day its ATX block changes.
    const auto iter = switchDays.find(Year_Doy_Number(day));
    if (iter == switchDays.end()) {
        return false;
    }
    return std::find(iter->second.begin(), iter->second.end(), prn) != iter->second.end();
}

bool Use_BDS_PRN_On_Day(const BDSSwitchDayPRNs& switchDays, const ProcessDay& day, int prn) {
    // Before 2026-04-10, keep the old rule and skip C01-C18.
    const int yearDoy = Year_Doy_Number(day);
    if (yearDoy < BDS_PRN_Renumbering_YearDoy && prn < 19) {
        return false;
    }
    if (yearDoy >= BDS_PRN_Renumbering_YearDoy
        && yearDoy <= BDS_PRN_Renumbering_EndYearDoy) {
        return !BDS_PRN_Switches_On_Day(switchDays, day, prn);
    }
    return true;
}

BDSOrbitType Get_BDS_Orbit_Type(const Antenna* antenna, int prn, bool strict = false) {
    // Use ATX antenna type for BDS orbit type.
    if (antenna != nullptr && !antenna->antennaType.empty()) {
        // Use only G/I/M orbit marks; ignore suffixes such as CAST or SECM.
        if (Contains_Text(antenna->antennaType, "BEIDOU-2G")
            || Contains_Text(antenna->antennaType, "BEIDOU-3G")) {
            return BDSOrbitType::GEO;
        }
        if (Contains_Text(antenna->antennaType, "BEIDOU-2I")
            || Contains_Text(antenna->antennaType, "BEIDOU-3I")
            || Contains_Text(antenna->antennaType, "BEIDOU-3SI")) {
            return BDSOrbitType::IGSO;
        }
        if (Contains_Text(antenna->antennaType, "BEIDOU-2M")
            || Contains_Text(antenna->antennaType, "BEIDOU-3M")
            || Contains_Text(antenna->antennaType, "BEIDOU-3SM")) {
            return BDSOrbitType::MEO;
        }
    }
    if (strict) {
        throw std::runtime_error("BDS PRN " + std::to_string(prn)
            + ": XYZ PCO requires a recognized ANTEX orbit type");
    }
    if (prn <= 10 || prn == 13 || prn == 15) {
        return BDSOrbitType::IGSO;
    }
    return BDSOrbitType::MEO;
}

double QZSS_Nominal_PCO(int prn) {
    // Return the nominal QZSS PCO.
    if (prn == 1) return 2.5457 * 3.2 - 1.5457 * 2.99;
    if (prn == 2) return 2.5457 * 2.394 - 1.5457 * 3.21;
    if (prn == 7) return 2.5457 * 2.962 - 1.5457 * 2.842;
    if (prn == 3) return 2.5457 * 2.387 - 1.5457 * 3.242;
    return 0.0;
}

double QZSS_Current_PCO(const Antenna* antenna) {
    // Return the current QZSS PCO.
    return 2.5457 * PCO_Up(antenna, 0) - 1.5457 * PCO_Up(antenna, 1);
}

double QZSS_Clock_PCO_Correction(const std::array<Antenna, constants::TotalSatNum>& antennas, int prn) {
    // Calculate the QZSS clock PCO correction.
    if (prn == 1) {
        return QZSS_Current_PCO(Get_Ant_Pointer(antennas, 'J', prn)) - QZSS_Nominal_PCO(prn);
    }
    if (prn == 2) {
        return QZSS_Current_PCO(Get_Ant_Pointer(antennas, 'J', prn)) - QZSS_Nominal_PCO(prn);
    }
    if (prn == 7) {
        return QZSS_Current_PCO(Get_Ant_Pointer(antennas, 'J', prn)) - QZSS_Nominal_PCO(prn);
    }
    if (prn == 3) {
        // Use antenna slot PRN+124.
        return QZSS_Current_PCO(Get_Ant_By_Index(antennas, prn + 124)) - QZSS_Nominal_PCO(prn);
    }
    return 0.0;
}

double SSR_APCPC_Radial_PCO(const std::array<Antenna, constants::TotalSatNum>& antennas, char system, int prn) {
    // Calculate APCPC radial PCO.
    const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);
    if (antenna == nullptr) {
        return 0.0;
    }
    if (system == 'C') {
        return 2.84 * PCO_Up(antenna, 0) - 1.84 * PCO_Up(antenna, 1);
    }
    if (system == 'E') {
        return 2.26 * PCO_Up(antenna, 0) - 1.26 * PCO_Up(antenna, 1);
    }
    return 2.5 * PCO_Up(antenna, 0) - 1.5 * PCO_Up(antenna, 1);
}

double SSR_APC_BDS_Radial_PCO(const std::array<Antenna, constants::TotalSatNum>& antennas,
                              char system, int prn, BDSFreq freq) {
    // Calculate BDS APC radial PCO for a named frequency combination.
    const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);
    if (antenna == nullptr || system != 'C') {
        return 0.0;
    }
    if (freq == BDSFreq::B1B2) {
        return 2.2606 * PCO_Up(antenna, "C02") - 1.2606 * PCO_Up(antenna, "C05");
    }
    return 2.94 * PCO_Up(antenna, "C02") - 1.94 * PCO_Up(antenna, "C06");
}

const Vector3* SSR_B1I_APC_PCO(const std::array<Antenna, constants::TotalSatNum>& antennas,
                              char system, int prn) {
    // Return the complete B1I antenna PCO vector from the active ATX block.
    const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);
    if (antenna == nullptr) {
        return nullptr;
    }
    const int frequencyIndex = system == 'C' ? PCO_Index(antenna, "C02") : 0;
    if (frequencyIndex < 0 || frequencyIndex >= antenna->pco.frequencyCount()) {
        return nullptr;
    }
    return &antenna->pco.at(frequencyIndex);
}

double SSR_IF_To_L1_Radial_PCO(const std::array<Antenna, constants::TotalSatNum>& antennas,
                               char system, int prn) {
    // Convert the legacy ionosphere-free radial PCO to the first-frequency PCO.
    const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);
    if (antenna == nullptr) {
        return 0.0;
    }
    if (system == 'C') {
        return -1.94 * PCO_Up(antenna, 0) + 1.94 * PCO_Up(antenna, 2);
    }
    if (system == 'E') {
        return -1.26 * PCO_Up(antenna, 0) + 1.26 * PCO_Up(antenna, 1);
    }
    return -1.5 * PCO_Up(antenna, 0) + 1.5 * PCO_Up(antenna, 1);
}

double Broadcast_Radial_PCO_Correction(const ProcessDay& day,
                                       const std::array<Antenna, constants::TotalSatNum>& antennas,
                                       char system, int prn);

double Clock_PCO_Correction(const Config& config, const ProcessDay& day,
                           const std::array<Antenna, constants::TotalSatNum>& antennas,
                           char system, int prn) {
    // Calculate clock PCO correction.
    if (!config.SSRRef.empty()) {
        return 0.0;
    }
    if (config.clktype[1] >= 1) {
        return 0.0;
    }

    const int yearDoy = day.year * 1000 + day.doy;
    const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);
    if (antenna == nullptr) {
        return 0.0;
    }

    if (system == 'G') {
        if (Contains_Text(antenna->antennaType, "BLOCK IIA")) return PCO_Up(antenna, 0) - 0.92;
        if (Contains_Text(antenna->antennaType, "BLOCK IIR-A")) return PCO_Up(antenna, 0) - 1.61;
        if (Contains_Text(antenna->antennaType, "BLOCK IIR-B")) return PCO_Up(antenna, 0) + 0.04;
        if (Contains_Text(antenna->antennaType, "BLOCK IIR-M")) return PCO_Up(antenna, 0) + 0.04;
        if (Contains_Text(antenna->antennaType, "BLOCK IIF")) return PCO_Up(antenna, 0) - 1.16;
        if (Contains_Text(antenna->antennaType, "BLOCK IIIA")) return PCO_Up(antenna, 0) - 1.16;
    } else if (system == 'R') {
        if (Contains_Text(antenna->antennaType, "GLONASS-K1")) return PCO_Up(antenna, 0) - 2.05;
        if (antenna->svn == 742) return PCO_Up(antenna, 0) - 1.95;
        if (antenna->svn >= 720) return PCO_Up(antenna, 0) - 2.05;
        if (antenna->svn <= 719) return PCO_Up(antenna, 0) - 2.45;
    } else if (system == 'E') {
        if (yearDoy < 2013121) return PCO_Up(antenna, 0) - 1.65;
        if (yearDoy < 2015060) return PCO_Up(antenna, 0) - 0.85;
        return PCO_Up(antenna, 0) - 0.75;
    } else if (system == 'C') {
        if (yearDoy < 2014197) {
            return 0.0;
        }
        if (Product_Uses_Brd(config, 1)) {
            return BDS_Broadcast_Clock_PCO(antenna, config.bdsfreq[0], yearDoy);
        }
        return 0.0;
    } else if (system == 'J') {
        return QZSS_Clock_PCO_Correction(antennas, prn);
    }

    return 0.0;
}

double Broadcast_Radial_PCO_Correction(const ProcessDay& day,
                                       const std::array<Antenna, constants::TotalSatNum>& antennas,
                                       char system, int prn) {
    // Calculate broadcast radial PCO correction.
    const int yearDoy = day.year * 1000 + day.doy;
    const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);

    if (system == 'G') {
        if (antenna == nullptr) return 0.0;
        if (Contains_Text(antenna->antennaType, "BLOCK IIA")) return 0.92;
        if (Contains_Text(antenna->antennaType, "BLOCK IIR-A")) return 1.61;
        if (Contains_Text(antenna->antennaType, "BLOCK IIR-B")) return -0.04;
        if (Contains_Text(antenna->antennaType, "BLOCK IIR-M")) return -0.04;
        if (Contains_Text(antenna->antennaType, "BLOCK IIF")) return 1.16;
        if (Contains_Text(antenna->antennaType, "BLOCK III")) return 1.16;
    } else if (system == 'R') {
        if (antenna == nullptr) return 0.0;
        if (Contains_Text(antenna->antennaType, "GLONASS-K1")) return 2.05;
        if (antenna->svn == 742) return 1.95;
        if (antenna->svn >= 720) return 2.05;
        if (antenna->svn <= 719) return 2.45;
    } else if (system == 'E') {
        if (yearDoy < 2015060) return 0.85;
        return 0.75;
    } else if (system == 'C') {
        if (yearDoy <= 2017016) return 0.0;
        if (yearDoy < 2024001) return BDS_Old_Brd_Radial_PCO(prn);
        return BDS_Current_Brd_Radial_PCO(antenna, yearDoy);
    } else if (system == 'J') {
        return QZSS_Nominal_PCO(prn);
    }

    return 0.0;
}

double Radial_PCO_Correction(const Config& config, const ProcessDay& day,
                             const std::array<Antenna, constants::TotalSatNum>& antennas,
                             char system, int prn) {
    // Calculate radial PCO correction.
    if (!config.SSRRef.empty()) {
        if (config.SSRRef == "COM") {
            return 0.0;
        }
        if (config.SSRRef == "APCPC") {
            return SSR_APCPC_Radial_PCO(antennas, system, prn);
        }
        if (config.SSRRef == "APCB1B2") {
            return SSR_APC_BDS_Radial_PCO(antennas, system, prn, BDSFreq::B1B2);
        }
        if (config.SSRRef == "APCB1B3") {
            return SSR_APC_BDS_Radial_PCO(antennas, system, prn, BDSFreq::B1B3);
        }
        if (config.SSRRef == "IFTOL1") {
            return SSR_IF_To_L1_Radial_PCO(antennas, system, prn);
        }
        return Broadcast_Radial_PCO_Correction(day, antennas, system, prn);
    }

    if (config.orbtype[1] == 1) {
        return 0.0;
    }
    return Broadcast_Radial_PCO_Correction(day, antennas, system, prn);
}

SISREValues Cal_SISRE(char system, int prn, const Antenna* antenna, const Vector3& dRTN, double dClk) {
    // Calculate SISRE values.
    SISREValues values;
    double radialScale = 0.98;
    double crossTrackDivisor = 49.0;
    if (system == 'C') {
        const BDSOrbitType orbitType = Get_BDS_Orbit_Type(antenna, prn);
        const bool geoOrIgso = orbitType == BDSOrbitType::GEO || orbitType == BDSOrbitType::IGSO;
        radialScale = geoOrIgso ? 0.99 : 0.98;
        crossTrackDivisor = geoOrIgso ? 126.0 : 54.0;
    } else if (system == 'R') {
        crossTrackDivisor = 45.0;
    } else if (system == 'E') {
        crossTrackDivisor = 61.0;
    } else if (system == 'J') {
        radialScale = 0.99;
        crossTrackDivisor = 126.0;
    }

    const double crossTrackTerm = (dRTN[0] * dRTN[0] + dRTN[1] * dRTN[1]) / crossTrackDivisor;
    values.sisre = std::sqrt(std::pow(radialScale * dRTN[2] - dClk, 2) + crossTrackTerm);
    values.orbit = std::sqrt(std::pow(radialScale * dRTN[2], 2) + crossTrackTerm);
    return values;
}

std::string Output_Header(const Config& config) {
    // Return the output header.
    if (config.do_orbit && config.do_clock) {
        return "PRN week sow dT dN dR dClk RminusClk SISRE SISRE_Orb";
    }
    if (config.do_orbit) {
        return "PRN week sow dT dN dR SISRE_Orb";
    }
    return "PRN week sow dClk";
}

OutputFiles Open_Output_Files(const Config& config, char system, int satNum) {
    // Open output files.
    std::filesystem::create_directories(config.OutDir);
    OutputFiles output;
    output.files.resize(static_cast<std::size_t>(satNum + 1));
    const std::string header = Output_Header(config);
    for (int prn = 1; prn <= satNum; ++prn) {
        const std::string outputPath = Get_SISRE_FileName(config, system, prn);
        std::error_code ignoredError;
        std::filesystem::remove(outputPath, ignoredError);
        output.files[static_cast<std::size_t>(prn)].open(outputPath);
        if (!output.files[static_cast<std::size_t>(prn)]) {
            throw std::runtime_error("failed to open output file: " + outputPath);
        }
        output.files[static_cast<std::size_t>(prn)] << header << "\n";
    }
    return output;
}

void Write_Result_Line(std::ostream& output, const Config& config, int prn, int obsWeek, double obsSec,
                     const Vector3& dRTN, double dClk, const SISREValues& sisre) {
    // Write one result line.
    output << std::setfill('0') << std::setw(2) << prn << std::setfill(' ')
           << std::setw(5) << obsWeek
           << std::setw(8) << static_cast<int>(obsSec)
           << std::fixed << std::setprecision(3);
    if (config.do_orbit && config.do_clock) {
        output << std::setw(10) << dRTN[0]
               << std::setw(10) << dRTN[1]
               << std::setw(10) << dRTN[2]
               << std::setw(10) << dClk
               << std::setw(10) << (dRTN[2] - dClk)
               << std::setw(10) << sisre.sisre
               << std::setw(10) << sisre.orbit
               << "\n";
    } else if (config.do_orbit) {
        output << std::setw(10) << dRTN[0]
               << std::setw(10) << dRTN[1]
               << std::setw(10) << dRTN[2]
               << std::setw(10) << sisre.orbit
               << "\n";
    } else {
        output << std::setw(10) << dClk << "\n";
    }
}

void Process_Epoch(const Config& config, const ProcessDay& day, const std::array<ProductRuntime, 2>& products,
                  const std::array<Antenna, constants::TotalSatNum>& antennas,
                  const std::array<Antenna, constants::TotalSatNum>* brdAntennas,
                  const BDSSwitchDayPRNs& bdsSwitchDayPRNs, OutputFiles& output,
                  ClkDatumState& clkDatumState, const SSRData* ssrData,
                  const std::vector<Vector3>* brdOrbCorr, char system, int satNum, int obsWeek, double obsSec,
                  std::vector<int>& lowBetaCount) {
    // Process one epoch.
    std::vector<Vector3> dRTN(static_cast<std::size_t>(satNum + 1));
    std::vector<bool> validOrbit(static_cast<std::size_t>(satNum + 1), false);
    std::vector<double> dClk(static_cast<std::size_t>(satNum + 1), 0.0);
    std::vector<bool> validClock(static_cast<std::size_t>(satNum + 1), false);

    for (int prn = 1; prn <= satNum; ++prn) {
        if (system == 'C' && !Use_BDS_PRN_On_Day(bdsSwitchDayPRNs, day, prn)) {
            continue;
        }

        const SatState Sat1 = Get_Sat_State(products[0], config, 0, system, prn, obsWeek, obsSec, nullptr);
        if (config.do_orbit && Is_Invalid_Position(Sat1.position)) {
            continue;
        }
        if (config.do_clock && Is_Invalid_Clock(Sat1.clock)) {
            continue;
        }

        const bool useFullSSRB1IAPC = config.ssrcorrtype == 2 && config.orbtype[1] == 2
            && config.SSRRef == "B1IAPC";
        SatState Sat2 = Get_Sat_State(products[1], config, 1, system, prn, obsWeek, obsSec, ssrData);
        if (config.do_orbit && Is_Invalid_Position(Sat2.position)) {
            continue;
        }
        if (config.do_clock && Is_Invalid_Clock(Sat2.clock)) {
            continue;
        }

        double brdPCO = 0.0;
        if (brdAntennas != nullptr) {
            brdPCO = BDS_Brd_PCO(Get_Ant_Pointer(antennas, system, prn),
                Get_Ant_Pointer(*brdAntennas, system, prn), prn);
        }

        if (config.do_orbit && config.orbtype[1] == 2) {
            if (config.ssrcorrtype == 1 && brdOrbCorr != nullptr
                && prn < static_cast<int>(brdOrbCorr->size())) {
                const Vector3& correction = (*brdOrbCorr)[static_cast<std::size_t>(prn)];
                for (int i = 0; i < 3; ++i) {
                    Sat2.position[static_cast<std::size_t>(i)] -= correction[static_cast<std::size_t>(i)];
                }
            } else if (config.ssrcorrtype == 2 && ssrData != nullptr) {
                const int PRNIndex = Get_PRNIndex(system, prn);
                if (!Has_SSR_Correction(ssrData, PRNIndex)) {
                    continue;
                }
                Sat2 = SSRCorr(PRNIndex, *ssrData, Sat2, SSR_Ref_Velocity(Sat1));
                if (useFullSSRB1IAPC) {
                    const Vector3* pco = SSR_B1I_APC_PCO(antennas, system, prn);
                    if (pco != nullptr) {
                        const Vector3 pcoEcef = Satellite_PCO_To_ECEF(
                            obsWeek, obsSec, products[1].navHead.leapSeconds,
                            Sat1.position, *pco);
                        for (int i = 0; i < 3; ++i) {
                            Sat2.position[static_cast<std::size_t>(i)]
                                -= pcoEcef[static_cast<std::size_t>(i)];
                        }
                    }
                }
            }
        }

        if (config.do_orbit) {
            const bool useBrdXYZ = brdAntennas != nullptr && config.brdPCO == "XYZ";
            if (useBrdXYZ) {
                const Antenna* antenna = Get_Ant_Pointer(*brdAntennas, system, prn);
                const Vector3& pco = antenna->pco.at(PCO_Index(antenna, "C06"));
                const bool isGeo = Get_BDS_Orbit_Type(antenna, prn, true) == BDSOrbitType::GEO;
                double betaDeg = 0.0;
                const Vector3 pcoEcef = BDS_PCO_To_ECEF(obsWeek, obsSec,
                    products[1].navHead.leapSeconds, Sat1.position, Sat1.velocity,
                    pco, isGeo, &betaDeg);
                if (!isGeo && std::abs(betaDeg) < 4.0) {
                    ++lowBetaCount[static_cast<std::size_t>(prn)];
                }
                // Convert broadcast APC to COM before differencing. Do not add radial PCO again.
                for (std::size_t i = 0; i < 3; ++i) {
                    Sat2.position[i] -= pcoEcef[i];
                }
            }
            const Vector3 dXYZ{
                Sat2.position[0] - Sat1.position[0],
                Sat2.position[1] - Sat1.position[1],
                Sat2.position[2] - Sat1.position[2],
            };
            if (Has_Huge_Component(dXYZ, 1000.0)) {
                continue;
            }
            dRTN[static_cast<std::size_t>(prn)] = XYZ2RTN(Sat1.position, Sat1.velocity, dXYZ);
            if (!useFullSSRB1IAPC && !useBrdXYZ) {
                dRTN[static_cast<std::size_t>(prn)][2]
                    += brdAntennas != nullptr ? brdPCO
                        : Radial_PCO_Correction(config, day, antennas, system, prn);
            }
            validOrbit[static_cast<std::size_t>(prn)] = true;
        }
        if (config.do_clock) {
            bool bdsFreqValid = true;
            const double bdsFreqCorrection =
                BDS_Clock_Frequency_Correction(config, products[1], system, prn, bdsFreqValid);
            if (!bdsFreqValid) {
                continue;
            }
            // Subtract the PCO difference between the two product reference points.
            const double dPCO = brdAntennas != nullptr
                ? BDS_Reference_Clock_PCO(Get_Ant_Pointer(antennas, system, prn),
                                          config.bdsfreq[0], prn) - brdPCO
                : Clock_PCO_Correction(config, day, antennas, system, prn);
            dClk[static_cast<std::size_t>(prn)] =
                (Sat2.clock + bdsFreqCorrection - Sat1.clock) * constants::SpeedOfLight - dPCO;
            validClock[static_cast<std::size_t>(prn)] = true;
        }
    }

    if (config.do_clock) {
        // Require three satellite clock-difference samples for datum.
        int validClockCount = 0;
        for (int prn = 1; prn <= satNum; ++prn) {
            if (validClock[static_cast<std::size_t>(prn)]) {
                ++validClockCount;
            }
        }
        if (validClockCount < 3) {
            return;
        }

        const bool useRminusClkDatum = Use_RminusClk_Datum(config);
        std::vector<double> datumValues = dClk;
        if (useRminusClkDatum) {
            // Use dClk-dR for R-C datum (from initial clock bias).
            for (int prn = 1; prn <= satNum; ++prn) {
                if (validClock[static_cast<std::size_t>(prn)]) {
                    datumValues[static_cast<std::size_t>(prn)]
                        -= dRTN[static_cast<std::size_t>(prn)][2];
                } else {
                    datumValues[static_cast<std::size_t>(prn)] = 0.0;
                }
            }
        }
        const double med = Cal_Clk_Datum(config, datumValues, satNum, clkDatumState);
        for (int prn = 1; prn <= satNum; ++prn) {
            if (!validClock[static_cast<std::size_t>(prn)]) {
                continue;
            }
            const double value = useRminusClkDatum
                ? dClk[static_cast<std::size_t>(prn)] - dRTN[static_cast<std::size_t>(prn)][2]
                : dClk[static_cast<std::size_t>(prn)];
            if (std::abs(value - med) > 999.0) {
                validClock[static_cast<std::size_t>(prn)] = false;
                dClk[static_cast<std::size_t>(prn)] = 0.0;
                datumValues[static_cast<std::size_t>(prn)] = 0.0;
            }
        }
        clkDatumState.PredClk = datumValues;
        for (int prn = 1; prn <= satNum; ++prn) {
            if (validClock[static_cast<std::size_t>(prn)]) {
                // Subtract datum (from initial clock bias).
                dClk[static_cast<std::size_t>(prn)] -= med;
            }
        }
    }

    for (int prn = 1; prn <= satNum; ++prn) {
        if (config.do_clock && !validClock[static_cast<std::size_t>(prn)]) {
            continue;
        }
        if (config.do_orbit && !validOrbit[static_cast<std::size_t>(prn)]) {
            continue;
        }
        const Antenna* antenna = Get_Ant_Pointer(antennas, system, prn);
        const SISREValues sisre = Cal_SISRE(system, prn, antenna, dRTN[static_cast<std::size_t>(prn)],
                                            dClk[static_cast<std::size_t>(prn)]);
        Write_Result_Line(output.files[static_cast<std::size_t>(prn)], config, prn, obsWeek, obsSec,
                        dRTN[static_cast<std::size_t>(prn)], dClk[static_cast<std::size_t>(prn)], sisre);
    }
}

int Run_Main(const Config& config, std::ostream& out, std::ostream& err) {
    // Run all systems and days.
    const std::vector<char> systems = Get_Process_Systems(config);
    const bool processBDS = std::find(systems.begin(), systems.end(), 'C') != systems.end();
    if (config.brdPCO == "XYZ" && (!processBDS || config.ssrcorrtype != 0
        || !Product_Uses_Brd(config, 1) || !config.do_orbit
        || Make_Process_Day(config.yearStart, config.doyStart).year < 2024)) {
        throw std::invalid_argument("brd_pco XYZ requires BDS broadcast orbit comparison "
                                    "from 2024, with SSR off");
    }
    if (processBDS && config.ssrcorrtype == 0 && Product_Uses_Brd(config, 1)
        && Make_Process_Day(config.yearStart, config.doyEnd).year >= 2024
        && config.BrdAntFile.empty()) {
        throw std::invalid_argument("BDS broadcast comparison from 2024 requires brd_antfile; "
            "keep antfile consistent with the precise reference product");
    }
    const BDSSwitchDayPRNs bdsSwitchDayPRNs = processBDS ? Read_BDS_Switch_Day_PRNs(config.AntFile) : BDSSwitchDayPRNs{};

    out << "mode=" << config.mode << "\n";
    out << "reference=" << config.productTypes[0] << " " << config.ACs[0] << "\n";
    out << "compared=" << config.productTypes[1] << " " << config.ACs[1] << "\n";
    if (processBDS) {
        out << "reference_bdsfreq=" << BDSFreq_Text(config.bdsfreq[0])
            << " compared_bdsfreq=" << BDSFreq_Text(config.bdsfreq[1]) << "\n";
        if (config.ssrcorrtype == 0 && Product_Uses_Brd(config, 1)) {
            out << "antfile=" << config.AntFile << "\n"
                << "brd_antfile=" << config.BrdAntFile << "\n";
            out << "brd_pco=" << config.brdPCO << "\n";
            if (config.brdPCO == "XYZ") {
                out << "BDS attitude: GEO orbit-normal; IGSO/MEO nominal yaw steering. "
                       "Special yaw maneuvers are not modeled.\n";
            }
        }
    }
    if (config.do_clock && (Product_Uses_Brd(config, 0) || Product_Uses_Brd(config, 1))) {
        out << "bds_brd_clock_corr="
            << BDS_Brd_Clock_Corr_Text(config.bdsBrdClockCorr) << "\n";
    }
    if (config.do_clock) {
        out << "clkdatum=" << Get_Clkdatum(config)
            << " clkdatum_method=" << config.clkdatumMethod << "\n";
    }
    if (config.ssrcorrtype != 0) {
        out << "ssrcorrtype=" << config.ssrcorrtype
            << " ssr_ref=" << (config.SSRRef.empty() ? "default" : config.SSRRef)
            << " ssrfile=" << (config.SSRFile.empty() ? "auto" : config.SSRFile)
            << " ssrdir=" << config.SSRDir
            << " ssr_format=" << config.SSRFormat << "\n";
    }

    for (const char system : systems) {
        // Process one satellite system.
        const int satNum = Get_SatNum(system);
        out << "system=" << system << " satellites=" << satNum << "\n";
        OutputFiles output = Open_Output_Files(config, system, satNum);
        std::array<Saved_Brd_NavData, 2> BrdNavData_Save;
        ClkDatumState clkDatumState;

        for (int inputDoy = config.doyStart; inputDoy <= config.doyEnd; ++inputDoy) {
            // Load files and antenna data for one day.
            const ProcessDay day = Make_Process_Day(config.yearStart, inputDoy);
            out << "  day=" << day.yearDoyString << " gps=" << day.gpsDayString << "\n";

            std::array<ProductRuntime, 2> products;
            try {
                const ProductFileNames files = Make_Product_FileNames(config, day);
                for (int product = 0; product < 2; ++product) {
                    ProductRuntime& runtime = products[static_cast<std::size_t>(product)];
                    Read_Product_Data(runtime, config, files, product, system, satNum);
                    if (Product_Uses_Brd(config, product)) {
                        Saved_Brd_NavData& NavData_Save = BrdNavData_Save[static_cast<std::size_t>(product)];
                        Append_Saved_Brd_NavData(runtime, NavData_Save, system, satNum);
                        Save_Brd_NavData(NavData_Save, runtime, system);
                    }
                }
            } catch (const std::exception& error) {
                err << "skip " << day.yearDoyString << " " << system << ": " << error.what() << "\n";
                continue;
            }
            const double mjd = UTC2MJD(day.year, 1, day.doy, 0, 0, 0.0);
            const auto antennas = Get_Ant(config.AntFile, mjd);
            std::array<Antenna, constants::TotalSatNum> brdAntennas{};
            const bool useBrdAntennas = system == 'C' && config.ssrcorrtype == 0
                && Product_Uses_Brd(config, 1) && day.year >= 2024;
            if (useBrdAntennas) {
                brdAntennas = Get_Ant(config.BrdAntFile, mjd);
            }
            SSRReader ssrReader;
            BrdOrbCorrReader brdOrbCorrReader;
            const SSRData* ssrData = nullptr;
            const std::vector<Vector3>* brdOrbCorr = nullptr;
            if (config.ssrcorrtype == 1) {
                const std::string ssrFile = Make_SSR_FileName(config, day);
                brdOrbCorrReader.Open(ssrFile, satNum);
                brdOrbCorr = &brdOrbCorrReader.data();
            } else if (config.ssrcorrtype == 2) {
                const std::string ssrFile = Make_SSR_FileName(config, day);
                ssrReader.Open(ssrFile);
                ssrData = &ssrReader.data();
            }

            const int obsWeek = day.gpsWeek;
            std::vector<int> lowBetaCount(static_cast<std::size_t>(satNum + 1), 0);
            double obsSec = day.gpsDay * 86400.0 + config.interval;
            while (true) {
                // Process the current epoch.
                obsSec += config.interval;
                if (obsSec >= (day.gpsDay + 1) * 86400.0 - 300.0) {
                    break;
                }

                Read_Next_Product_Data(products[0], config, 0, obsWeek, obsSec);
                Read_Next_Product_Data(products[1], config, 1, obsWeek, obsSec);
                if (config.ssrcorrtype == 1) {
                    brdOrbCorrReader.Read_BrdOrbCorr(system, obsWeek, obsSec);
                } else if (config.ssrcorrtype == 2) {
                    ssrReader.Get_SSR(obsWeek, obsSec, config.SSRAge);
                    if (!Has_SSR_Epoch(ssrData, obsWeek, obsSec, config.SSRAge)) {
                        continue;
                    }
                }
                Process_Epoch(config, day, products, antennas, useBrdAntennas ? &brdAntennas : nullptr,
                              bdsSwitchDayPRNs, output, clkDatumState,
                              ssrData, brdOrbCorr, system, satNum, obsWeek, obsSec, lowBetaCount);
            }
            for (int prn = 1; prn <= satNum; ++prn) {
                if (lowBetaCount[static_cast<std::size_t>(prn)] > 0) {
                    err << "warning " << day.yearDoyString << " " << system << prn
                        << ": nominal PCO attitude used at " << lowBetaCount[static_cast<std::size_t>(prn)]
                        << " epochs with |beta| < 4 deg; verify yaw maneuvers\n";
                }
            }
        }
    }

    return 0;
}

}  // namespace orbclkcmp
