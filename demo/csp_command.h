#ifndef CSP_COMMAND_H
#define CSP_COMMAND_H

#include <cstdint>

constexpr int CSP_MOTOR_COUNT = 6;
constexpr const char *CSP_SOCKET_PATH = "/tmp/csp.sock";

// One complete native-endian Unix datagram. positions[0] maps to EtherCAT slave 1.
// Units are absolute encoder counts, matching the PDO target position.
struct CspCommand {
    int32_t positions[CSP_MOTOR_COUNT];
};
static_assert(sizeof(CspCommand) == 6 * sizeof(int32_t), "Unexpected command padding");

// Send these four bytes from a bound socket to subscribe without commanding motion.
constexpr char CSP_SUBSCRIBE[4] = {'C', 'S', 'U', '1'};
constexpr uint32_t CSP_FEEDBACK_READY = 1;

// Native-endian, same-host protocol; Rust decodes fields explicitly.
struct CspFeedback {
    char magic[4];                 // "CSF1"
    uint32_t cycle;
    int32_t positions[CSP_MOTOR_COUNT];
    uint16_t statuswords[CSP_MOTOR_COUNT];
    uint32_t flags;                // bit 0: all drives ready for motion in EtherCAT OP
};
static_assert(sizeof(CspFeedback) == 48, "Unexpected feedback padding");

#endif
