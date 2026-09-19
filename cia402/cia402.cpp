#include "cia402.h"

uint16_t cia402_decode_state(uint16_t status_word) {
    return status_word & SW_STATE_MASK;
}

uint16_t cia402_control_word(uint16_t status_word) {
    uint16_t control_word;

    // bit mask to read only the bits displaying state information
    uint16_t masked_state = status_word & SW_STATE_MASK;

    switch (masked_state) {
        case SW_STATE_NOT_READY_TO_SWITCH_ON:
            control_word = 0x0000;  // safe control word
            break;
        case SW_STATE_SWITCH_ON_DISABLED:
            control_word = CW_SHUTDOWN_CMD;
            break;
        case SW_STATE_READY_TO_SWITCH_ON:
            control_word = CW_SWITCH_ON_CMD;
            break;
        case SW_STATE_SWITCHED_ON:
            control_word = CW_ENABLE_OP_CMD;
            break;
        case SW_STATE_OPERATION_ENABLED:
            control_word = CW_ENABLE_OP_CMD;
            break;
        case SW_STATE_QUICK_STOP_ACTIVE:
            control_word = CW_QUICK_STOP_CMD;   // enable voltage to allow motor to perform quick stop
            break;
        case SW_STATE_FAULT_REACTION_ACTIVE:
            control_word = CW_QUICK_STOP_CMD;   // enable voltage to allow motor to perform fault reaction
            break;
        case SW_STATE_FAULT:
            control_word = CW_FAULT_RESET_CMD;
            break;
        default:
            control_word = 0x0000;
            break;
    }

    return control_word;
}