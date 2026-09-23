// CiA 402 Object dictionary constants shared by all control modes

#ifndef _CIA402_H_
#define _CIA402_H_

#include <stdint.h>

// Indexes
#define INDEX_CONTROL_WORD          0x6040U
#define INDEX_STATUS_WORD           0x6041U
#define INDEX_OP_MODE               0x6060U
#define INDEX_OP_MODE_DISPLAY       0x6061U
#define INDEX_TARGET_POSITION       0x607AU
#define INDEX_ACTUAL_POSITION       0x6064U
#define INDEX_TARGET_VELOCITY       0x60FFU
#define INDEX_ACTUAL_VELOCITY       0x606CU
#define INDEX_MAX_VELOCITY          0x6080U
#define INDEX_PROFILE_VELOCITY      0x6081U
#define INDEX_PROFILE_ACCEL         0x6083U
#define INDEX_PROFILE_DECEL         0x6084U
#define INDEX_QUICK_STOP_DECEL      0x6085U
#define INDEX_MAX_ACCELERATION      0x60C5U
#define INDEX_ERROR_CODE            0x603FU

// Mode of operation
#define MODE_PROFILE_POSITION       ((int8_t)1)
#define MODE_PROFILE_VELOCITY       ((int8_t)3)
#define MODE_CSP                    ((int8_t)8)
#define MODE_CSV                    ((int8_t)9)

// Control word bits
#define CW_BITS_SWITCH_ON           (1U << 0)
#define CW_BITS_ENABLE_VOLTAGE      (1U << 1)
#define CW_BITS_QUICK_STOP          (1U << 2)
#define CW_BITS_ENABLE_OPERATION    (1U << 3)
#define CW_BITS_NEW_SETPOINT        (1U << 4) // PP mode only
#define CW_BITS_IMMEDIATE_UPDATE    (1U << 5) // PP mode only
#define CW_BITS_FAULT_RESET         (1U << 7)
#define CW_BITS_HALT                (1U << 8)

// Status word bits
#define SW_BITS_READY_TO_SWITCH_ON      (1U << 0)
#define SW_BITS_SWITCHED_ON             (1U << 1)
#define SW_BITS_OPERATION_ENABLED       (1U << 2)
#define SW_BITS_FAULT                   (1U << 3)
#define SW_BITS_VOLTAGE_ENABLED         (1U << 4)
#define SW_BITS_QUICK_STOP              (1U << 5)
#define SW_BITS_SWITCH_ON_DISABLED      (1U << 6)
#define SW_BITS_WARNING                 (1U << 7)
#define SW_BITS_TARGET_REACHED          (1U << 10)
#define SW_BITS_INTERNAL_LIMIT_ACTIVE   (1U << 11)
#define SW_BITS_SETPOINT_ACK            (1U << 12)

// CiA 402 state machine sequence (fault reset --> operation enable)
#define CW_FAULT_RESET_CMD          (CW_BITS_FAULT_RESET)   // 0x0080
#define CW_QUICK_STOP_CMD           (CW_BITS_ENABLE_VOLTAGE)    // 0x0002
#define CW_SHUTDOWN_CMD             (CW_BITS_ENABLE_VOLTAGE | CW_BITS_QUICK_STOP) // 0x0006
#define CW_SWITCH_ON_CMD            (CW_SHUTDOWN_CMD | CW_BITS_SWITCH_ON) // 0x0007
#define CW_ENABLE_OP_CMD            (CW_SWITCH_ON_CMD | CW_BITS_ENABLE_OPERATION) // 0x000F

// Status word state mask
#define SW_STATE_MASK               0x006FU

// CiA 402 status word states
#define SW_STATE_NOT_READY_TO_SWITCH_ON    0x0000U
#define SW_STATE_SWITCH_ON_DISABLED        0x0040U
#define SW_STATE_READY_TO_SWITCH_ON        0x0021U
#define SW_STATE_SWITCHED_ON               0x0023U
#define SW_STATE_OPERATION_ENABLED         0x0027U
#define SW_STATE_QUICK_STOP_ACTIVE         0x0007U
#define SW_STATE_FAULT_REACTION_ACTIVE     0x000FU
#define SW_STATE_FAULT                     0x0008U

typedef enum {
    NOT_READY_TO_SWITCH_ON = SW_STATE_NOT_READY_TO_SWITCH_ON,
    SW_ON_DISABLED = SW_STATE_SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON = SW_STATE_READY_TO_SWITCH_ON,
    SWITCHED_ON = SW_STATE_SWITCHED_ON,
    OPERATION_ENABLED = SW_STATE_OPERATION_ENABLED,
    QUICK_STOP_ACTIVE = SW_STATE_QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE = SW_STATE_FAULT_REACTION_ACTIVE,
    FAULT = SW_STATE_FAULT,
    UNKNOWN = 0xFFFFU
} CIA402_STATE;

// Structure for RXPDO (Control data sent to slave)
typedef struct {
    uint16_t controlword;      // 0x6040:0, 16 bits
    int32_t target_velocity;   // 0x60FF:0, 32 bits, only used in CSV, PV
    uint8_t mode_of_operation; // 0x6060:0, 8 bits
    uint8_t padding;           // 8 bits padding for alignment
} __attribute__((__packed__)) rxpdo_t;

// Structure for TXPDO (Status data received from slave)
typedef struct {
    uint16_t statusword;      // 0x6041:0, 16 bits
    int32_t actual_position;  // 0x6064:0, 32 bits
    int32_t actual_velocity;  // 0x606C:0, 32 bits
    int16_t actual_torque;    // 0x6077:0, 16 bits
} __attribute__((__packed__)) txpdo_t;

typedef struct {
    CIA402_STATE state;
    rxpdo_t rxpdo;
    txpdo_t txpdo;
    uint32_t step;
    bool faulted;
    bool operation_enabled;
} cia402_motor_t;

/**
 * Decodes status word received from motor
 * @param {uint16_t} status_word: raw status word
 * @return {enum CIA402_STATE}: CiA 402 state enum
 */
CIA402_STATE cia402_decode_state(uint16_t status_word);


/**
 * Returns control word according to status word
 * @param {enum CIA402_STATE} state: current state of slave
 * @return {uint16_t}: control word to transition to next state
 */
uint16_t cia402_control_word(CIA402_STATE state);

/**
 * Initialises cia402_motor_t motor's RXPDO and state variables
 * @param {cia402_motor_t *}: motor, {const int}: mode_of_operation
 * @return {void}
 */
void cia402_init_motor(cia402_motor_t *motor, const int mode_of_operation);

/**
 * CiA 402 state machine control  
 * Sets `faulted` and `operation_enabled` state variables
 * @param {cia402_motor_t *}: motor
 * @return {void}
 */
void cia402_state_machine(cia402_motor_t *motor);

#endif
