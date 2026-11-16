#include "pdo.h"
#include "cia402.h"
#include "sdo.h"

namespace {

// RXPDO: control data sent to slave, must match rxpdo_t
constexpr pdo_entry_t CSV_RXPDO[] = {
    {INDEX_CONTROL_WORD,    0x00, 16},
    {INDEX_TARGET_VELOCITY, 0x00, 32},
    {INDEX_OP_MODE,         0x00, 8},
    {0x0000,                0x00, 8},   // padding
};

// TXPDO: status data received from slave, must match txpdo_t
constexpr pdo_entry_t CSV_TXPDO[] = {
    {INDEX_STATUS_WORD,     0x00, 16},
    {INDEX_ACTUAL_POSITION, 0x00, 32},
    {INDEX_ACTUAL_VELOCITY, 0x00, 32},
    {0x6077,                0x00, 16},  // actual torque
};

template <size_t N>
constexpr size_t pdo_bytes(const pdo_entry_t (&entries)[N]) {
    size_t bits = 0;
    for (size_t i = 0; i < N; i++) {
        bits += entries[i].bit_length;
    }
    return bits / 8;
}

static_assert(pdo_bytes(CSV_RXPDO) == sizeof(rxpdo_t), "CSV RXPDO table does not match rxpdo_t");
static_assert(pdo_bytes(CSV_TXPDO) == sizeof(txpdo_t), "CSV TXPDO table does not match txpdo_t");

template <size_t N>
constexpr uint8 count_of(const pdo_entry_t (&)[N]) {
    return static_cast<uint8>(N);
}

} // namespace

bool pdo_write_mapping(uint16 slave, uint16 pdo_index, uint16 assign_index,
                       const pdo_entry_t *entries, uint8 count) {
    // Clear the mapping object
    if (!sdo_write_value<uint8>(slave, pdo_index, 0x00, 0)) return false;

    // Write each mapped object as 0xIIIISSLL (index, subindex, bit length)
    for (uint8 i = 0; i < count; i++) {
        uint32 map_object = ((uint32)entries[i].index << 16) |
                            ((uint32)entries[i].subindex << 8) |
                            entries[i].bit_length;
        if (!sdo_write_value<uint32>(slave, pdo_index, i + 1, map_object)) return false;
    }

    // Set number of mapped objects
    if (!sdo_write_value<uint8>(slave, pdo_index, 0x00, count)) return false;

    // Assign the mapping object to the sync manager
    if (!sdo_write_value<uint16>(slave, assign_index, 0x00, 0x0000)) return false;
    if (!sdo_write_value<uint16>(slave, assign_index, 0x01, pdo_index)) return false;
    if (!sdo_write_value<uint16>(slave, assign_index, 0x00, 0x0001)) return false;

    return true;
}

bool pdo_map_csv_rxpdo(uint16 slave) {
    return pdo_write_mapping(slave, 0x1600, 0x1C12, CSV_RXPDO, count_of(CSV_RXPDO));
}

bool pdo_map_csv_txpdo(uint16 slave) {
    return pdo_write_mapping(slave, 0x1A00, 0x1C13, CSV_TXPDO, count_of(CSV_TXPDO));
}
