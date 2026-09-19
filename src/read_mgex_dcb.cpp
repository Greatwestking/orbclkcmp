#include "read_mgex_dcb.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>

#include "standard_reader.hpp"

namespace orbclkcmp {
namespace {

bool Has_Text(const std::string& line, const std::string& text) {
    return line.find(text) != std::string::npos;
}

bool Read_Code_Pair_Diff(const std::string& line, std::initializer_list<const char*> firstCodes,
                         std::initializer_list<const char*> secondCodes, double value, double& diff) {
    for (const char* first : firstCodes) {
        for (const char* second : secondCodes) {
            const std::size_t firstPos = line.find(first);
            const std::size_t secondPos = line.find(second);
            if (firstPos == std::string::npos || secondPos == std::string::npos) {
                continue;
            }
            diff = firstPos < secondPos ? value : -value;
            return true;
        }
    }
    return false;
}

bool Is_DCB_Line(const std::string& line) {
    const std::string type = field(line, 1, 4);
    return type.find("DSB") != std::string::npos || type.find("DCB") != std::string::npos
        || type.find("OSB") != std::string::npos;
}

bool Is_OSB_Line(const std::string& line) {
    return field(line, 1, 4).find("OSB") != std::string::npos;
}

void Fill_99(std::array<std::array<double, 9>, constants::CNum + 1>& values) {
    for (auto& row : values) {
        row.fill(99.0);
    }
}

void Save_BDS_OSB(const std::string& line, double value, std::array<double, 9>& row) {
    if (Has_Text(line, "C2I")) row[0] = value;
    else if (Has_Text(line, "C6I")) row[1] = value;
    else if (Has_Text(line, "C7I")) row[2] = value;
    else if (Has_Text(line, "C1P")) row[3] = value;
    else if (Has_Text(line, "C1X")) row[4] = value;
    else if (Has_Text(line, "C1D")) row[5] = value;
    else if (Has_Text(line, "C5P")) row[6] = value;
    else if (Has_Text(line, "C5X")) row[7] = value;
    else if (Has_Text(line, "C5D")) row[8] = value;
}

bool Has_Bias(double value) {
    return std::abs(value - 99.0) > 1.0e-12;
}

double Bias_Diff(double first, double second) {
    return first - second;
}

double First_Bias(const std::array<double, 9>& row, std::initializer_list<int> indexes) {
    for (int index : indexes) {
        if (Has_Bias(row[static_cast<std::size_t>(index)])) {
            return row[static_cast<std::size_t>(index)];
        }
    }
    return 99.0;
}

double Read_Bias_Value(const std::string& line) {
    const std::size_t unitPos = line.find(" ns ");
    if (unitPos != std::string::npos) {
        std::istringstream input(line.substr(unitPos + 4));
        double value = 0.0;
        if (input >> value) {
            return value * 1.0e-9;
        }
    }
    return readDouble(line, 81, 92) * 1.0e-9;
}

}  // namespace

MGEXDCB::MGEXDCB() {
    for (auto& row : bds) {
        row.fill(99.0);
    }
    for (auto& row : glonass) {
        row.fill(99.0);
    }
}

MGEXDCB Read_MGEXDCB(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open DCB file: " + path);
    }

    std::string line;
    bool inBiasBlock = false;
    while (std::getline(input, line)) {
        if (line.find("*BIAS") != std::string::npos && line.find("SVN_") != std::string::npos) {
            inBiasBlock = true;
            break;
        }
    }
    if (!inBiasBlock) {
        throw std::runtime_error("DCB file has no *BIAS block: " + path);
    }

    MGEXDCB dcb;
    std::array<std::array<double, 9>, constants::CNum + 1> bdsOsb{};
    Fill_99(bdsOsb);
    while (std::getline(input, line)) {
        if (line.find("-BIAS/SOLUTION") != std::string::npos) {
            break;
        }
        if (field(line, 16, 19) != "    ") {
            continue;
        }
        if (!Is_DCB_Line(line)) {
            continue;
        }

        const char system = field(line, 12, 12)[0];
        if (system != 'C' && system != 'R') {
            continue;
        }

        const int prn = readInt(line, 13, 14);
        const double value = Read_Bias_Value(line);

        if (system == 'C') {
            if (prn < 1 || prn > constants::CNum) {
                continue;
            }
            double diff = 0.0;
            if (Is_OSB_Line(line)) {
                Save_BDS_OSB(line, value, bdsOsb[static_cast<std::size_t>(prn)]);
            } else if (Read_Code_Pair_Diff(line, {"C2I"}, {"C7I"}, value, diff)) {
                dcb.bds[static_cast<std::size_t>(prn)][2] = diff;
            } else if (Read_Code_Pair_Diff(line, {"C2I"}, {"C6I"}, value, diff)) {
                dcb.bds[static_cast<std::size_t>(prn)][0] = diff;
            } else if (Read_Code_Pair_Diff(line, {"C7I"}, {"C6I"}, value, diff)) {
                dcb.bds[static_cast<std::size_t>(prn)][1] = diff;
            } else if (Read_Code_Pair_Diff(line, {"C1P", "C1X", "C1D"}, {"C6I"}, value, diff)) {
                dcb.bds[static_cast<std::size_t>(prn)][3] = diff;
            } else if (Read_Code_Pair_Diff(line, {"C5P", "C5X", "C5D"}, {"C6I"}, value, diff)) {
                dcb.bds[static_cast<std::size_t>(prn)][4] = diff;
            } else if (Read_Code_Pair_Diff(line, {"C1P", "C1X", "C1D"}, {"C5P", "C5X", "C5D"}, value, diff)) {
                dcb.bds[static_cast<std::size_t>(prn)][5] = diff;
            }
        } else if (system == 'R') {
            if (prn < 1 || prn > constants::RNum) {
                continue;
            }
            if (Has_Text(line, "C1P  C2P")) {
                dcb.glonass[static_cast<std::size_t>(prn)][0] = value;
            } else if (Has_Text(line, "C1C  C1P")) {
                dcb.glonass[static_cast<std::size_t>(prn)][1] = value;
            }
        }
    }

    for (int prn = 1; prn <= constants::CNum; ++prn) {
        auto& row = dcb.bds[static_cast<std::size_t>(prn)];
        const auto& osb = bdsOsb[static_cast<std::size_t>(prn)];
        if (!Has_Bias(row[0]) && Has_Bias(osb[0]) && Has_Bias(osb[1])) {
            row[0] = Bias_Diff(osb[0], osb[1]);
        }
        if (!Has_Bias(row[1]) && Has_Bias(osb[2]) && Has_Bias(osb[1])) {
            row[1] = Bias_Diff(osb[2], osb[1]);
        }
        if (!Has_Bias(row[2]) && Has_Bias(osb[0]) && Has_Bias(osb[2])) {
            row[2] = Bias_Diff(osb[0], osb[2]);
        }
        const double b1c = First_Bias(osb, {3, 4, 5});
        const double b2a = First_Bias(osb, {6, 7, 8});
        if (!Has_Bias(row[3]) && Has_Bias(b1c) && Has_Bias(osb[1])) {
            row[3] = Bias_Diff(b1c, osb[1]);
        }
        if (!Has_Bias(row[4]) && Has_Bias(b2a) && Has_Bias(osb[1])) {
            row[4] = Bias_Diff(b2a, osb[1]);
        }
        if (!Has_Bias(row[5]) && Has_Bias(b1c) && Has_Bias(b2a)) {
            row[5] = Bias_Diff(b1c, b2a);
        }
        if (!Has_Bias(row[4]) && Has_Bias(row[3]) && Has_Bias(row[5])) {
            row[4] = row[3] - row[5];
        }
        if (!Has_Bias(row[3]) && Has_Bias(row[4]) && Has_Bias(row[5])) {
            row[3] = row[4] + row[5];
        }
        if (!Has_Bias(row[5]) && Has_Bias(row[3]) && Has_Bias(row[4])) {
            row[5] = row[3] - row[4];
        }
        if (row[2] != 99.0 && row[0] != 99.0 && row[1] == 99.0) {
            row[1] = row[0] - row[2];
        }
    }

    return dcb;
}

}  // namespace orbclkcmp
