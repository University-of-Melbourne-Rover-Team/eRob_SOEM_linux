/**
 * URT EtherCAT maindevice configuration file
 */

#ifndef URT_CONFIG_H
#define URT_CONFIG_H

#include <cstdint>
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
constexpr int PROCESS_PRIORITY = 99;
constexpr int PROCESS_CPU_CORES[] = {2, 3};

// Stack size for the RT and check threads
constexpr int THREAD_STACK_SIZE = 128 * 1024;

// Slave state monitoring timeout used by the check thread (us)
constexpr int EC_TIMEOUTMON = 5000;

// Drive parameters written in step 9
constexpr int8_t OPERATION_MODE = MODE_CSV;
constexpr int32_t MAX_VELOCITY = 30000;
constexpr int32_t MAX_ACCELERATION = 30000;
constexpr int32_t QUICK_STOP_DECEL = 30000;

// Fixed target velocity applied to every enabled motor
constexpr int32_t DEMO_TARGET_VELOCITY = -2000;

// Print TXPDO data every N cycles
constexpr int TELEMETRY_PRINT_INTERVAL = 100;

}

#endif
