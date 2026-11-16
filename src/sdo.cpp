#include "sdo.h"

#include <cstdio>

bool sdo_write(uint16 slave, uint16 index, uint8 subindex, int size, void *data) {
    // ec_SDOwrite returns the mailbox working counter: > 0 on success, 0 on SDO abort or timeout
    int wkc = ec_SDOwrite(slave, index, subindex, FALSE, size, data, EC_TIMEOUTSAFE);
    if (wkc > 0) {
        return true;
    }

    printf("ERROR: SDO write failed: slave %d, 0x%04x:%02x (%d bytes)\n", slave, index, subindex, size);
    // An SDO abort is pushed onto the SOEM error list, print and pop every entry
    while (ec_iserror()) {
        printf("       %s", ec_elist2string());
    }
    return false;
}
