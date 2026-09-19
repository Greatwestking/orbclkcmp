#include "prn_index.hpp"

#include "constants.hpp"

namespace orbclkcmp {

int Get_PRNIndex(char system, int prn) {
    if (system == 'G') {
        if (prn < 1 || prn > constants::GNum) {
            return -1;
        }
        return prn - 1;
    }
    if (system == 'R') {
        if (prn < 1 || prn > constants::RNum) {
            return -1;
        }
        return constants::GNum + prn - 1;
    }
    if (system == 'C') {
        if (prn < 1 || prn > constants::CNum) {
            return -1;
        }
        return constants::GNum + constants::RNum + prn - 1;
    }
    if (system == 'E') {
        if (prn < 1 || prn > constants::NumE) {
            return -1;
        }
        return constants::GNum + constants::RNum + constants::CNum + prn - 1;
    }
    if (system == 'J') {
        if (prn < 1 || prn > constants::JNum) {
            return -1;
        }
        return constants::GNum + constants::RNum + constants::CNum + constants::NumE + prn - 1;
    }
    return -1;
}

}  // namespace orbclkcmp
