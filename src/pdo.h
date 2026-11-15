/**
 * PDO mapping tables and SDO writer
 * The packed rxpdo_t / txpdo_t structs (cia402.h) must match the tables in pdo.cpp
 */

#ifndef URT_PDO_H
#define URT_PDO_H

#include <cstdint>
#include "ethercat.h"

typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
} pdo_entry_t;

/**
 * Writes a PDO mapping object (0x16xx / 0x1Axx) and assigns it to a sync manager (0x1C12 / 0x1C13)
 * @param {uint16} slave, {uint16} pdo_index, {uint16} assign_index, {const pdo_entry_t *} entries, {uint8} count
 * @return {int}: sum of SDO write working counters
 */
int pdo_write_mapping(uint16 slave, uint16 pdo_index, uint16 assign_index,
                      const pdo_entry_t *entries, uint8 count);

/**
 * Writes the CSV RXPDO (0x1600 -> 0x1C12) mapping to a slave
 * @return {int}: sum of SDO write working counters
 */
int pdo_map_csv_rxpdo(uint16 slave);

/**
 * Writes the CSV TXPDO (0x1A00 -> 0x1C13) mapping to a slave
 * @return {int}: sum of SDO write working counters
 */
int pdo_map_csv_txpdo(uint16 slave);

#endif
