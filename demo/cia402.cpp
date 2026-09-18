#include "cia402.h"

DriveState cia402_decode_state(uint16_t status) {
    DriveState result;
    uint16_t masked_state = status & SW_STATE_MASK;

    switch (masked_state) {
        case SW_STATE_NOT_READY_TO_SWITCH_ON:
            result.type = DRIVE_STATE_NOT_READY_TO_SWITCH_ON;
            result.unknown_value = 0;
            break;
        case SW_STATE_SWITCH_ON_DISABLED:
            result.type = DRIVE_STATE_SWITCH_ON_DISABLED;
            result.unknown_value = 0;
            break;
        case SW_STATE_READY_TO_SWITCH_ON:
            result.type = DRIVE_STATE_READY_TO_SWITCH_ON;
            result.unknown_value = 0;
            break;
        case SW_STATE_SWITCHED_ON:
            result.type = DRIVE_STATE_SWITCHED_ON;
            result.unknown_value = 0;
            break;
        case SW_STATE_OPERATION_ENABLED:
            result.type = DRIVE_STATE_OPERATION_ENABLED;
            result.unknown_value = 0;
            break;
        case SW_STATE_QUICK_STOP_ACTIVE:
            result.type = DRIVE_STATE_QUICK_STOP_ACTIVE;
            result.unknown_value = 0;
            break;
        case SW_STATE_FAULT_REACTION_ACTIVE:
            result.type = DRIVE_STATE_FAULT_REACTION_ACTIVE;
            result.unknown_value = 0;
            break;
        case SW_STATE_FAULT:
            result.type = DRIVE_STATE_FAULT;
            result.unknown_value = 0;
            break;
        default:
            result.type = DRIVE_STATE_UNKNOWN;
            result.unknown_value = masked;
            break;
    }

    return result;
}