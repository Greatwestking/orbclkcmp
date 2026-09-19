#pragma once

#include <array>
#include <fstream>
#include <string>
#include <vector>

#include "constants.hpp"
#include "types.hpp"

namespace orbclkcmp {

enum class SSRFileType {
    SSR,
    BNC,
};

class SSRReader {
public:
    void Open(const std::string& path);
    void Get_SSR(int obsWeek, double obsSow, double ssrAge);
    const SSRData& data() const;

private:
    std::ifstream input_;
    std::string pendingLine_;
    SSRFileType type_ = SSRFileType::SSR;
    SSRData data_;
};

class BrdOrbCorrReader {
public:
    void Open(const std::string& path, int satNum);
    void Read_BrdOrbCorr(char system, int obsWeek, double obsSow);
    const std::vector<Vector3>& data() const;

private:
    std::ifstream input_;
    std::string pendingLine_;
    std::vector<Vector3> data_;
};

// Get the existing SSR direction velocity from a reference SP3 ECEF state.
Vector3 SSR_Ref_Velocity(const SatState& reference);

SatState SSRCorr(int PRNIndex, const SSRData& ssr, const SatState& state,
                 const Vector3& directionVelocity);

}  // namespace orbclkcmp
