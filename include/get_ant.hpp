#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "constants.hpp"
#include "types.hpp"

namespace orbclkcmp {

// Read satellite antenna PCO data for the requested MJD.
std::array<Antenna, constants::TotalSatNum> Get_Ant(const std::string& path, double mjd);

using BDSSwitchDayPRNs = std::map<int, std::vector<int>>;

// Read BDS PRNs with an ATX validity change on each YYYYDDD day. Because the BDS new PRNs.
BDSSwitchDayPRNs Read_BDS_Switch_Day_PRNs(const std::string& path);

}  // namespace orbclkcmp
