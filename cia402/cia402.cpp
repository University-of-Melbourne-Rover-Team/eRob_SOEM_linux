#include "cia402.h"


// CiA 402 decodes the status word received from the motor and returns the corresponding state
CIA402_STATE cia402_decode_state(uint16_t status_word) {
    uint16_t masked_status_word = status_word & SW_STATE_MASK;
    switch(masked_status_word) {
        case SW_STATE_NOT_READY_TO_SWITCH_ON:
            return NOT_READY_TO_SWITCH_ON;
        case SW_STATE_SWITCH_ON_DISABLED:
            return SW_ON_DISABLED;
        case SW_STATE_READY_TO_SWITCH_ON:
            return READY_TO_SWITCH_ON;
        case SW_STATE_SWITCHED_ON:
            return SWITCHED_ON;
        case SW_STATE_OPERATION_ENABLED:
            return OPERATION_ENABLED;
        case SW_STATE_QUICK_STOP_ACTIVE:
            return QUICK_STOP_ACTIVE;
        case SW_STATE_FAULT_REACTION_ACTIVE:
            return FAULT_REACTION_ACTIVE;
        case SW_STATE_FAULT:
            return FAULT;
        default:
            return UNKNOWN;
    }
}

uint16_t cia402_control_word(CIA402_STATE state) {
    uint16_t control_word;

    switch (state) {
        case NOT_READY_TO_SWITCH_ON:
            control_word = 0x0000;  // safe control word
            break;
        case SW_ON_DISABLED:
            control_word = CW_SHUTDOWN_CMD;
            break;
        case READY_TO_SWITCH_ON:
            control_word = CW_SWITCH_ON_CMD;
            break;
        case SWITCHED_ON:
            control_word = CW_ENABLE_OP_CMD;
            break;
        case OPERATION_ENABLED:
            control_word = CW_ENABLE_OP_CMD;
            break;
        case QUICK_STOP_ACTIVE:
            control_word = CW_QUICK_STOP_CMD;   // enable voltage to allow motor to perform quick stop
            break;
        case FAULT_REACTION_ACTIVE:
            control_word = CW_QUICK_STOP_CMD;   // enable voltage to allow motor to perform fault reaction
            break;
        case FAULT:
            control_word = CW_FAULT_RESET_CMD;
            break;
        case UNKNOWN:
            control_word = 0x0000;
        default:
            control_word = 0x0000;
            break;
    }

    return control_word;
}

void cia402_init_motor(cia402_motor_t *motor, const int mode_of_operation) {
    // initialise RXPDO
    motor->rxpdo.controlword = CW_FAULT_RESET_CMD;
    motor->rxpdo.mode_of_operation = mode_of_operation;
    motor->rxpdo.padding = 0;

    // initialise motor state variables
    motor->state = NOT_READY_TO_SWITCH_ON;
    motor->faulted = false;
    motor->operation_enabled = false;
    motor->step = 0;
}

void cia402_state_machine(cia402_motor_t *motor) {

    // get motor state from TXPDO status word
    motor->state = cia402_decode_state(motor->txpdo.statusword);

    // check for unknown state
    if (motor->state == UNKNOWN) {
        motor->rxpdo.controlword = 0x0000U; // safe control word
        motor->operation_enabled = false;
        motor->step = 0;    // reset state machine progression
        return;
    }

    // check for fault reaction
    if (motor->state == QUICK_STOP_ACTIVE || motor->state == FAULT_REACTION_ACTIVE) {
        motor->rxpdo.controlword = CW_QUICK_STOP_CMD; // safe control word
        motor->operation_enabled = false;
        motor->step = 0;    // reset state machine progression
        return;
    }

    /**
     * Assign control word and state variables based on state
     */

    // not ready to switch on || fault state: send Fault reset command
    if (((motor->state == NOT_READY_TO_SWITCH_ON) || (motor->state == FAULT))) {
        motor->faulted = true;
        motor->operation_enabled = false;
        motor->rxpdo.controlword = CW_FAULT_RESET_CMD;
        motor->step = 0;    // reset state machine progression

        /**
         * TO DO: Implement fault reset rising edge
         */
    }

    // switch on disabled state: send shutdown command
    else if (motor->state == SW_ON_DISABLED) {
        motor->faulted = false;
        motor->operation_enabled = false;

        if (motor->step >= 500) {
            motor->rxpdo.controlword = CW_SHUTDOWN_CMD;
            motor->step = 0;
        }
    }

    // ready to switch on state: send switch on command
    else if (motor->state == READY_TO_SWITCH_ON) {
        motor->faulted = false;
        motor->operation_enabled = false;

        if (motor->step >= 500) {
            motor->rxpdo.controlword = CW_SWITCH_ON_CMD;
            motor->step = 0;
        }
    }

    // switched on state: send enable operation command
    else if (motor->state == SWITCHED_ON) {
        motor->faulted = false;
        motor->operation_enabled = false;

        if (motor->step >= 500) {
            motor->rxpdo.controlword = CW_ENABLE_OP_CMD;
            motor->step = 0;
        }
    }

    // operation enabled state: send enable operation command
    else if (motor->state == OPERATION_ENABLED) {
        motor->faulted = false;
        motor->operation_enabled = true;
        motor->rxpdo.controlword = CW_ENABLE_OP_CMD;
    }

    if (motor->step < 2000) {
        motor->step++;
    }
}