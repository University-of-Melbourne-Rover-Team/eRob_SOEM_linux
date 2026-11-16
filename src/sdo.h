/**
 * Service Data Object handler
 */

#ifndef URT_SDO_H
#define URT_SDO_H

#include "ethercat.h"

#include <cstdint>

/**
 * Writes one SDO object and reports failure
 * @param {uint16} slave, {uint16} index, {uint8} subindex, {int} size: bytes, {void *} data
 * @return {bool}: true if slave acknowledged the write
 */
bool sdo_write(uint16 slave, uint16 index, uint8 subindex, int size, void *data);

/**
 * Writes one SDO object of type T
 */
template <typename T>
bool sdo_write_value(uint16 slave, uint16 index, uint8 subindex, T value) {
    return sdo_write(slave, index, subindex, sizeof(T), &value);
}

#endif