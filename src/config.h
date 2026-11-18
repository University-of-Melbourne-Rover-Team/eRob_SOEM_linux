/**
 * URT EtherCAT maindevice configuration file
 */

#ifndef URT_CONFIG_H
#define URT_CONFIG_H

#include <cstdint>
#include <cstddef>
#include <cstdint>
#include <iterator>

#include "cia402.h"
#include "utils.h"

namespace config {

/**
 * EtherCAT Master configuration:
 */

// Default EtherCAT network interface (run ip link)
constexpr const char *DEFAULT_IF_NAME = "enp89s0";  // if_name of NUC

// EtherCAT cycle time; SYNC0 is configured with the same period
constexpr int CYCLE_TIME_US = 1000; // PDO's exchanged periodically every 1000us = 1ms
constexpr uint32_t SYNC0_CYCLE_NS = CYCLE_TIME_US * 1000; // cycle time of Sync 0 event
// offset of Sync 0 event (i.e. Sync 0 event occurs 500us after Sync 0 trigger)
constexpr int32_t SYNC0_SHIFT_NS = SYNC0_CYCLE_NS / 2;

// Process real-time settings
constexpr int PROCESS_CPU_CORES[] = {2, 3};

// Thread configuration (SCHED_FIFO policy)
// 1-99, higher runs first. Main thread stays on SCHED_OTHER policy
constexpr int RT_THREAD_PRIORITY = 90;  // real-time ethercat loop thread
constexpr int CHECK_THREAD_PRIORITY = 20;   // ecat comms status check/recover thread
constexpr int THREAD_STACK_SIZE = 128 * 1024; // Stack size for the RT and check threads

// EtherCAT bus state monitoring thread config (ecat_check)
// Slave state monitoring timeout used by the check thread (us)
constexpr int EC_TIMEOUTMON = 5000;

// ecat check thread config
constexpr int CHECK_PERIOD_US = 10000;  // polling period
constexpr int WKC_ERROR_THRESHOLD = 5;  // wkc error counts before slave states are checked
constexpr int WKC_REPORT_INTERVAL = 100; // wkc error warning interval


/**
 * EtherCAT Fieldbus configuration:
 */

// Maximum number of motors allowed on the bus
constexpr int MAX_MOTORS = 16;

// CiA 402 mode of operation for each motor in bus order (entry 0 = slave 1)
// MODE_CSV and MODE_CSP supported
constexpr int8_t MOTOR_MODES[] = {MODE_CSV, MODE_CSV, MODE_CSV, MODE_CSV, MODE_CSP, MODE_CSV};
constexpr int NUM_MOTORS = static_cast<int>(std::size(MOTOR_MODES));

constexpr bool motor_modes_supported() {
    for (int8_t mode : MOTOR_MODES) {
        if (mode != MODE_CSV && mode != MODE_CSP) {
            return false;
        }
    }
    return true;
}

static_assert(NUM_MOTORS <= MAX_MOTORS, "MOTOR_MODES has more entries than MAX_MOTORS");
static_assert(motor_modes_supported(), "MOTOR_MODES contain mode other than MODE_CSV or MODE_CSP");

// Motor internal limits
constexpr double MAX_VELOCITY_DEG = 60.0;   // deg/s
constexpr double MAX_ACCELERATION_DEG = 60.0;   // deg/s^2
constexpr double QUICK_STOP_DECEL_DEG = 60.0;   // deg/s^2

constexpr int32_t MAX_VELOCITY = MAX_VELOCITY_DEG / EROB_DEG_PER_COUNT; // counts/s
constexpr int32_t MAX_ACCELERATION = MAX_ACCELERATION_DEG / EROB_DEG_PER_COUNT; // counts/s^2
constexpr int32_t QUICK_STOP_DECEL = QUICK_STOP_DECEL_DEG / EROB_DEG_PER_COUNT; // counts/s^2

// CSP trapezoidal profile values used by trajectory planner
constexpr double CSP_PROFILE_VELOCITY_DEG = 20.0;   // deg/s
constexpr double CSP_PROFILE_ACCEL_DEG = 20.0;      // deg/s^2
constexpr double CSP_PROFILE_VELOCITY = CSP_PROFILE_VELOCITY_DEG / EROB_DEG_PER_COUNT;   // counts/s
constexpr double CSP_PROFILE_ACCELERATION = CSP_PROFILE_ACCEL_DEG / EROB_DEG_PER_COUNT;  // counts/s^2

static_assert(CSP_PROFILE_VELOCITY > 0 && CSP_PROFILE_VELOCITY <= MAX_VELOCITY,
              "CSP_PROFILE_VELOCITY must be > 0 and <= drive MAX_VELOCITY");
static_assert(CSP_PROFILE_ACCELERATION > 0 && CSP_PROFILE_ACCELERATION <= MAX_ACCELERATION,
              "CSP_PROFILE_ACCELERATION must be > 0 and <= drive MAX_ACCELERATION");

// demo values
constexpr double DEMO_CSP_SWING_DEG[] = {0.0, 0.0, 0.0, 0.0, 90.0, 0.0};
static_assert(std::size(DEMO_CSP_SWING_DEG) == NUM_MOTORS, "DEMO_CSP_SWING_DEG is wrong size");
constexpr double DEMO_CSV_VELOCITY_DEG[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static_assert(std::size(DEMO_CSV_VELOCITY_DEG) == NUM_MOTORS, "DEMO_CSV_VELOCITY_DEG is wrong size");

// Print TXPDO data every N cycles
constexpr int TELEMETRY_PRINT_INTERVAL = 250;

}

#endif
