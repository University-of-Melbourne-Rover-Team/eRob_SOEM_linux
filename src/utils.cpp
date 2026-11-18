#include "utils.h"

double count2angle(int32_t count) {
    return count * EROB_DEG_PER_COUNT;
}

int32_t angle2count(double angle) {
    return static_cast<int32_t>(std::lround(angle / EROB_DEG_PER_COUNT));
}
