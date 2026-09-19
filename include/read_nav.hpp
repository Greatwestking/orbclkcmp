#pragma once

#include <istream>
#include <string>
#include <vector>

#include "types.hpp"

namespace orbclkcmp {

struct NavRecordEntry {
    char system = '\0';
    int prn = 0;
    NavRecord record;
};

struct GlonassNavRecordEntry {
    int prn = 0;
    GlonassNavRecord record;
};

// Read a RINEX navigation header.
NavHead ReadNavHead(std::istream& input);
NavHead ReadNavHeadFile(const std::string& path);

// Read RINEX v2/v3/v4 broadcast ephemeris blocks.
// IsCNAV selects the v4 CNAV/CNV1 branches.
std::vector<NavRecordEntry> ReadNavData(std::istream& input, char system, const NavHead& head);
std::vector<NavRecordEntry> ReadNavData(std::istream& input, char system, const NavHead& head,
                                        const RuntimeOptions& options);
std::vector<NavRecordEntry> ReadNavDataFile(const std::string& path, char system, const NavHead& head);
std::vector<NavRecordEntry> ReadNavDataFile(const std::string& path, char system, const NavHead& head,
                                            const RuntimeOptions& options);

// Read RINEX v2/v3/v4 GLONASS navigation records.
std::vector<GlonassNavRecordEntry> ReadNavData_R(std::istream& input, const NavHead& head);
std::vector<GlonassNavRecordEntry> ReadNavData_RFile(const std::string& path, const NavHead& head);

}  // namespace orbclkcmp
