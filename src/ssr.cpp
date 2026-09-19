#include "ssr.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "cross.hpp"
#include "prn_index.hpp"
#include "standard_reader.hpp"
#include "time_convert.hpp"

namespace orbclkcmp {
namespace {

double GPS_Time_Diff(int weekA, double sowA, int weekB, double sowB) {
    return (weekA - weekB) * 604800.0 + sowA - sowB;
}

std::vector<std::string> Split_Text(const std::string& line) {
    std::istringstream input(line);
    std::vector<std::string> values;
    std::string value;
    while (input >> value) {
        values.push_back(value);
    }
    return values;
}

Vector3 Zero_Vector() {
    return {0.0, 0.0, 0.0};
}

double Vector_Norm(const Vector3& value) {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

Vector3 Normalize_Vector(const Vector3& value) {
    const double norm = Vector_Norm(value);
    if (norm <= 0.0) {
        return Zero_Vector();
    }
    return {value[0] / norm, value[1] / norm, value[2] / norm};
}

int Parse_SSR_PRN_Index(char system, int prn) {
    const int PRNIndex = Get_PRNIndex(system, prn);
    if (PRNIndex < 0) {
        return -1;
    }
    return PRNIndex;
}

bool Parse_Sat_Text(const std::string& text, char& system, int& prn) {
    if (text.size() < 2) {
        return false;
    }
    system = text[0];
    try {
        prn = std::stoi(text.substr(1));
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

void Store_SSR_Record(SSRData& data, char system, int prn, int iode, const Vector3& dOrb, double dClk) {
    const int PRNIndex = Parse_SSR_PRN_Index(system, prn);
    if (PRNIndex < 0) {
        return;
    }
    data.iode[static_cast<std::size_t>(PRNIndex)] = static_cast<double>(iode);
    data.dOrb[static_cast<std::size_t>(PRNIndex)] = dOrb;
    data.dClk[static_cast<std::size_t>(PRNIndex)] = dClk;
}

void Store_SSR_Orbit(SSRData& data, char system, int prn, int iode, const Vector3& dOrb) {
    const int PRNIndex = Parse_SSR_PRN_Index(system, prn);
    if (PRNIndex < 0) {
        return;
    }
    data.iode[static_cast<std::size_t>(PRNIndex)] = static_cast<double>(iode);
    data.dOrb[static_cast<std::size_t>(PRNIndex)] = dOrb;
}

void Store_SSR_Clock(SSRData& data, char system, int prn, double dClk) {
    const int PRNIndex = Parse_SSR_PRN_Index(system, prn);
    if (PRNIndex < 0) {
        return;
    }
    data.dClk[static_cast<std::size_t>(PRNIndex)] = dClk;
}

bool Read_Pending_Or_Line(std::ifstream& input, std::string& pendingLine, std::string& line) {
    if (!pendingLine.empty()) {
        line = pendingLine;
        pendingLine.clear();
        return true;
    }
    return static_cast<bool>(std::getline(input, line));
}

SSRFileType Detect_SSR_Type(const std::string& firstLine) {
    if (firstLine.find('>') != std::string::npos) {
        return SSRFileType::BNC;
    }
    return SSRFileType::SSR;
}

void Read_SSR_Text_Line(const std::string& line, SSRData& data) {
    const int week = readInt(line, 1, 4);
    const double sow = readDouble(line, 5, 11);
    const char system = readText(line, 14, 14).empty() ? '\0' : readText(line, 14, 14)[0];
    const int prn = readInt(line, 15, 16);
    const int iode = readInt(line, 23, 25);
    const double dClk = readDouble(line, 89, 96);
    Vector3 dOrb{};
    dOrb[2] = readDouble(line, 35, 42);
    dOrb[0] = readDouble(line, 44, 51);
    dOrb[1] = readDouble(line, 53, 60);
    data.week = week;
    data.sow = sow;
    Store_SSR_Record(data, system, prn, iode, dOrb, dClk);
}

}  // namespace

void SSRReader::Open(const std::string& path) {
    input_.open(path);
    if (!input_) {
        throw std::runtime_error("failed to open SSR file: " + path);
    }

    std::string firstLine;
    if (std::getline(input_, firstLine)) {
        type_ = Detect_SSR_Type(firstLine);
        data_.isBNC = type_ == SSRFileType::BNC;
        pendingLine_ = firstLine;
    }
}

void SSRReader::Get_SSR(int obsWeek, double obsSow, double ssrAge) {
    std::string line;
    while (Read_Pending_Or_Line(input_, pendingLine_, line)) {
        if (type_ == SSRFileType::SSR) {
            if (line.empty() || line[0] == '%') {
                continue;
            }

            int week = 0;
            double sow = 0.0;
            try {
                week = readInt(line, 1, 4);
                sow = readDouble(line, 5, 11);
            } catch (const std::exception&) {
                continue;
            }

            const double dt = GPS_Time_Diff(week, sow, obsWeek, obsSow);
            if (dt > 0.0) {
                pendingLine_ = line;
                break;
            }
            if (dt < -ssrAge) {
                continue;
            }

            Read_SSR_Text_Line(line, data_);
            continue;
        }

        if (line.find("> ") == std::string::npos) {
            continue;
        }

        const std::vector<std::string> words = Split_Text(line);
        if (words.size() < 10) {
            continue;
        }

        const std::string type = words[1];
        const int year = std::stoi(words[2]);
        const int month = std::stoi(words[3]);
        const int day = std::stoi(words[4]);
        const int hour = std::stoi(words[5]);
        const int minute = std::stoi(words[6]);
        const double second = std::stod(words[7]);
        const int prnCount = std::stoi(words[9]);
        const GpstTime gpst = UTC2GPST(year, month, day, hour, minute, second);
        const double dt = GPS_Time_Diff(gpst.week, gpst.seconds, obsWeek, obsSow);
        if (dt > 0.0) {
            pendingLine_ = line;
            break;
        }
        if (dt < -ssrAge) {
            continue;
        }

        if (type == "ORBIT") {
            for (int i = 0; i < prnCount && std::getline(input_, line); ++i) {
                std::istringstream row(line);
                std::string satText;
                int iode = 0;
                Vector3 raw{};
                row >> satText >> iode >> raw[0] >> raw[1] >> raw[2];
                char system = '\0';
                int prn = 0;
                if (!row || !Parse_Sat_Text(satText, system, prn)) {
                    continue;
                }
                Vector3 dOrb{};
                dOrb[2] = raw[0];
                dOrb[0] = raw[1];
                dOrb[1] = raw[2];
                data_.week = gpst.week;
                data_.sow = gpst.seconds;
                Store_SSR_Orbit(data_, system, prn, iode, dOrb);
            }
        } else if (type == "CLOCK") {
            for (int i = 0; i < prnCount && std::getline(input_, line); ++i) {
                std::istringstream row(line);
                std::string satText;
                int iode = 0;
                double dClk = 0.0;
                row >> satText >> iode >> dClk;
                char system = '\0';
                int prn = 0;
                if (!row || !Parse_Sat_Text(satText, system, prn)) {
                    continue;
                }
                data_.week = gpst.week;
                data_.sow = gpst.seconds;
                Store_SSR_Clock(data_, system, prn, dClk);
            }
        }
    }
}

const SSRData& SSRReader::data() const {
    return data_;
}

void BrdOrbCorrReader::Open(const std::string& path, int satNum) {
    input_.open(path);
    if (!input_) {
        throw std::runtime_error("failed to open orbit correction file: " + path);
    }
    data_.assign(static_cast<std::size_t>(satNum + 1), Zero_Vector());
}

void BrdOrbCorrReader::Read_BrdOrbCorr(char system, int obsWeek, double obsSow) {
    std::string line;
    while (Read_Pending_Or_Line(input_, pendingLine_, line)) {
        if (line.size() < 3 || line[0] != system) {
            continue;
        }

        int prn = 0;
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        double second = 0.0;
        int iode = 0;
        Vector3 correction{};
        try {
            prn = std::stoi(line.substr(1, 2));
            std::istringstream row(line.substr(5));
            row >> year >> month >> day >> hour >> minute >> second >> iode
                >> correction[0] >> correction[1] >> correction[2];
            if (!row) {
                continue;
            }
        } catch (const std::exception&) {
            continue;
        }

        const GpstTime gpst = UTC2GPST(year, month, day, hour, minute, second);
        if (GPS_Time_Diff(gpst.week, gpst.seconds, obsWeek, obsSow) > 0.01) {
            pendingLine_ = line;
            break;
        }
        if (prn >= 1 && prn < static_cast<int>(data_.size())) {
            data_[static_cast<std::size_t>(prn)] = correction;
        }
    }
}

const std::vector<Vector3>& BrdOrbCorrReader::data() const {
    return data_;
}

Vector3 SSR_Ref_Velocity(const SatState& reference) {
    // Preserve the existing SSR direction rule, including the low-speed fallback.
    Vector3 velocity = reference.velocity;
    if (Vector_Norm(velocity) < 500.0) {
        const double earthRotation = std::sin(constants::Wgs84EarthAngularVelocity);
        velocity[0] -= earthRotation * reference.position[1];
        velocity[1] += earthRotation * reference.position[0];
    }
    return velocity;
}

SatState SSRCorr(int PRNIndex, const SSRData& ssr, const SatState& state,
                 const Vector3& directionVelocity) {
    SatState corrected = state;
    if (PRNIndex < 0 || PRNIndex >= constants::TotalSatNum) {
        return corrected;
    }

    const Vector3 along = Normalize_Vector(directionVelocity);
    const Vector3 initialRadial = Normalize_Vector(state.position);
    const Vector3 cross = Cross(initialRadial, along);
    const Vector3 radial = Cross(along, cross);
    const Vector3& dOrb = ssr.dOrb[static_cast<std::size_t>(PRNIndex)];

    for (int i = 0; i < 3; ++i) {
        corrected.position[static_cast<std::size_t>(i)] -=
            along[static_cast<std::size_t>(i)] * dOrb[0] +
            cross[static_cast<std::size_t>(i)] * dOrb[1] +
            radial[static_cast<std::size_t>(i)] * dOrb[2];
    }
    corrected.clock += ssr.dClk[static_cast<std::size_t>(PRNIndex)] / constants::SpeedOfLight;
    return corrected;
}

}  // namespace orbclkcmp
