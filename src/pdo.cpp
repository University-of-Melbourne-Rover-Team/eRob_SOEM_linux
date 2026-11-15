#include "pdo.h"
#include "cia402.h"

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

int pdo_write_mapping(uint16 slave, uint16 pdo_index, uint16 assign_index,
                      const pdo_entry_t *entries, uint8 count) {
    int retval = 0;

    // Clear the mapping object
    uint8 zero_map = 0;
    retval += ec_SDOwrite(slave, pdo_index, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);

    // Write each mapped object as 0xIIIISSLL (index, subindex, bit length)
    for (uint8 i = 0; i < count; i++) {
        uint32 map_object = ((uint32)entries[i].index << 16) |
                            ((uint32)entries[i].subindex << 8) |
                            entries[i].bit_length;
        retval += ec_SDOwrite(slave, pdo_index, i + 1, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
    }

    // Set number of mapped objects
    retval += ec_SDOwrite(slave, pdo_index, 0x00, FALSE, sizeof(count), &count, EC_TIMEOUTSAFE);

    // Assign the mapping object to the sync manager
    uint16 assign_val = 0x0000;
    retval += ec_SDOwrite(slave, assign_index, 0x00, FALSE, sizeof(assign_val), &assign_val, EC_TIMEOUTSAFE);
    assign_val = pdo_index;
    retval += ec_SDOwrite(slave, assign_index, 0x01, FALSE, sizeof(assign_val), &assign_val, EC_TIMEOUTSAFE);
    assign_val = 0x0001;
    retval += ec_SDOwrite(slave, assign_index, 0x00, FALSE, sizeof(assign_val), &assign_val, EC_TIMEOUTSAFE);

    return retval;
}

int pdo_map_csv_rxpdo(uint16 slave) {
    return pdo_write_mapping(slave, 0x1600, 0x1C12, CSV_RXPDO, count_of(CSV_RXPDO));
}

int pdo_map_csv_txpdo(uint16 slave) {
    return pdo_write_mapping(slave, 0x1A00, 0x1C13, CSV_TXPDO, count_of(CSV_TXPDO));
}
