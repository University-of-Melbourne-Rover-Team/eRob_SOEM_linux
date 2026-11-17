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

namespace config {

// EtherCAT network interface used when none is given on the command line (run `ip link`)
constexpr const char *DEFAULT_IF_NAME = "enp89s0";  // if_name of NUC

// EtherCAT cycle time; SYNC0 is configured with the same period
constexpr int CYCLE_TIME_US = 1000; // PDO's exchanged periodically every 1000us = 1ms
constexpr uint32_t SYNC0_CYCLE_NS = CYCLE_TIME_US * 1000;

// Upper bound on slaves handled by the maindevice
constexpr int MAX_MOTORS = 16;

// Process real-time settings
constexpr int PROCESS_CPU_CORES[] = {2, 3};

// SCHED_FIFO policy thread priorities
// 1-99, higher runs first. Main thread stays on SCHED_OTHER policy
constexpr int RT_THREAD_PRIORITY = 90;  // real-time ethercat loop thread
constexpr int CHECK_THREAD_PRIORITY = 20;   // ecat comms status check/recover thread

// Stack size for the RT and check threads
constexpr int THREAD_STACK_SIZE = 128 * 1024;

// Slave state monitoring timeout used by the check thread (us)
constexpr int EC_TIMEOUTMON = 5000;

// ecat check thread config
constexpr int CHECK_PERIOD_US = 10000;  // polling period
constexpr int WKC_ERROR_THRESHOLD = 5;  // wkc error counts before slave states are checked
constexpr int WKC_REPORT_INTERVAL = 100; // wkc error warning interval

// CiA 402 mode of operation for each motor in bus order (entry 0 = slave 1)
// must list exactly one entry per slave on bus, otherwise program fails
// MODE_CSV and MODE_CSP supported
constexpr int8_t MOTOR_MODES[] = {MODE_CSV, MODE_CSV, MODE_CSV, MODE_CSV, MODE_CSV, MODE_CSV};
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

// Drive parameters
constexpr int32_t MAX_VELOCITY = 30000;
constexpr int32_t MAX_ACCELERATION = 30000;
constexpr int32_t QUICK_STOP_DECEL = 30000;

// target velocity for CSV motors
constexpr int32_t DEMO_TARGET_VELOCITY = 0;

// Print TXPDO data every N cycles
constexpr int TELEMETRY_PRINT_INTERVAL = 250;

}

#endif
