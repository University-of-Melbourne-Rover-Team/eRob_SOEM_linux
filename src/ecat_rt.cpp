#include "ecat_rt.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <pthread.h>

#include "ethercat.h"
#include "cia402.h"
#include "config.h"
#include "ecat_master.h"
#include "rt_utils.h"
#include "sdo.h"
#include "utils.h"
#include "trajectory.h"

namespace {

pthread_t ecatrt_thread;
pthread_t ecat_check_thread;
int cycle_time_us_param;

// Motor state, index 0 unused so motors[i] matches ec_slave[i]
cia402_motor_t motors[config::MAX_MOTORS + 1];

// per-slave OP status, true = slave in OP
std::atomic<bool> slave_in_op[config::MAX_MOTORS + 1];

// set by RT thread when a drive enters fault, cleared by check thread once the error code is read
std::atomic<bool> error_code_request[config::MAX_MOTORS + 1];

// true while the drive reports fault reaction active or fault (RT thread only)
bool drive_fault[config::MAX_MOTORS + 1];

// Cyclic sync position control data structures
trajectory_t trajectories[config::MAX_MOTORS + 1];
const trajectory_limits_t CSP_LIMITS = {config::CSP_MAX_VELOCITY, config::CSP_MAX_ACCELERATION};

/**
 * Updates connection state, CiA 402 state machine and targets for every motor (one RT cycle)
 */
void update_motors(double cycle_time_s) {
    /**
     * Check the connection status of each slave on the bus using working counter
     * Each slave expected to increment working counter 3 times each
     * i.e. 10 slaves --> wkc == 30
     */
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        bool slave_connected = slave_in_op[slave].load(std::memory_order_relaxed);
        // motor just disconnected (falling edge)
        if (motors[slave].ec_connected && !slave_connected) {
            cia402_lost_motor(&motors[slave]);
            memcpy(ec_slave[slave].outputs, &(motors[slave].rxpdo), sizeof(rxpdo_t));
        }
        motors[slave].ec_connected = slave_connected;
    }

    for (int slave = 1; slave <= ec_slavecount; slave++) {
        // skip disconnected slaves
        if (!motors[slave].ec_connected) {
            continue;
        }

        // copy TXPDO data into local array and update state machine
        memcpy(&(motors[slave].txpdo), ec_slave[slave].inputs, sizeof(txpdo_t));
        bool motor_faulted = motors[slave].faulted;
        cia402_state_machine(&motors[slave]);

        // motor just faulted
        if (!motor_faulted && motors[slave].faulted) {
            printf("FAULT: Slave %d, SW=0x%04x\n", slave, motors[slave].txpdo.statusword);
        }

        // drive just entered fault: ask check thread to read the error code
        bool in_fault = motors[slave].state == FAULT || motors[slave].state == FAULT_REACTION_ACTIVE;
        if (in_fault && !drive_fault[slave]) {
            error_code_request[slave].store(true, std::memory_order_relaxed);
        }
        drive_fault[slave] = in_fault;

        // set targets
        cia402_motor_t &motor = motors[slave];
        trajectory_t &trajectory = trajectories[slave];
        if (!motor.operation_enabled) {
            trajectory_reset(&trajectory, motor.txpdo.actual_position);
            motor.rxpdo.target_velocity = 0;
            motor.rxpdo.target_position = motor.txpdo.actual_position;
        }
        else if (motor.rxpdo.mode_of_operation == MODE_CSP) {
            if (trajectory_goal_reached(&trajectory)) {
                int32_t current_pos = trajectory.position;
                int32_t target_pos = current_pos + angle2count(config::DEMO_TARGET_POSITION);
                trajectory_set_goal(&trajectory, target_pos);
            }
            motor.rxpdo.target_position = trajectory_step(&trajectory, &CSP_LIMITS, cycle_time_s);
            motor.rxpdo.target_velocity = 0;    // unused in CSP
        }
        else if (motor.rxpdo.mode_of_operation == MODE_CSV) {
            motor.rxpdo.target_velocity = config::DEMO_TARGET_VELOCITY;
            motor.rxpdo.target_position = motor.txpdo.actual_position;  // unused in CSV
        }

        // copy RXPDO data from local array to SOEM array
        memcpy(ec_slave[slave].outputs, &(motors[slave].rxpdo), sizeof(rxpdo_t));
    }
}

/**
 * Prints WKC and TXPDO / RXPDO data for every motor
 * TODO(step 5): move out of the RT thread
 */
void print_motors() {
    printf("WKC: %d | expected WKC: %d\n", wkc, expectedWKC);
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        
        // CSV motor print
        if (motors[slave].rxpdo.mode_of_operation == MODE_CSV) {
            printf("Motor %d: SW=0x%04x, CW=0x%04x, pos=%.2f, vel=%.2f, target_vel=%d, mode=%s, fault=%d, op_en=%d, ec_connected=%d\n", slave,
               motors[slave].txpdo.statusword,
               motors[slave].rxpdo.controlword,
               count2angle(motors[slave].txpdo.actual_position),
               count2angle(motors[slave].txpdo.actual_velocity),
               motors[slave].rxpdo.target_velocity,
               cia402_mode2string(motors[slave].rxpdo.mode_of_operation),
               motors[slave].faulted,
               motors[slave].operation_enabled,
               motors[slave].ec_connected);
        }

        // CSP motor print
        else if (motors[slave].rxpdo.mode_of_operation == MODE_CSP) {
            printf("Motor %d: SW=0x%04x, CW=0x%04x, pos=%.2f, vel=%.2f, target_pos=%d, target_reached=%d, mode=%s, fault=%d, op_en=%d, ec_connected=%d\n", slave,
               motors[slave].txpdo.statusword,
               motors[slave].rxpdo.controlword,
               count2angle(motors[slave].txpdo.actual_position),
               count2angle(motors[slave].txpdo.actual_velocity),
               trajectories[slave].goal,
               trajectory_goal_reached(&trajectories[slave]),
               cia402_mode2string(motors[slave].rxpdo.mode_of_operation),
               motors[slave].faulted,
               motors[slave].operation_enabled,
               motors[slave].ec_connected);
        }
    }
    printf("\n");
}

/**
 * RT EtherCAT thread
 * Sends and receives process data every cycle, synchronised to the distributed clock
 */
void *ecatthread(void *ptr) {
    const int cycle_time_us = *(int *)ptr;
    const int64_t cycletime = (int64_t)cycle_time_us * 1000;
    const int MAX_MISSED_CYCLES = 10;

    struct timespec ts, tleft, cycle_start, cycle_end;
    int64_t toff = 0;
    int missed_cycles = 0;
    long cycle_count = 0;

    // align first wake-up to the next whole millisecond
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec = ((ts.tv_nsec / 1'000'000) + 1) * 1'000'000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }

    // configure motors on startup
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        cia402_init_motor(&motors[slave], config::MOTOR_MODES[slave-1]);
    }

    // send initial RXPDO data
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        memcpy(ec_slave[slave].outputs, &(motors[slave].rxpdo), sizeof(rxpdo_t));
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);

        // TODO(step 4): clock_nanosleep does not report missed deadlines, compare against ts instead
        add_timespec(&ts, cycletime + toff);
        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft) != 0) {
            missed_cycles++;
            if (missed_cycles >= MAX_MISSED_CYCLES) {
                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;
                if (ts.tv_nsec >= NSEC_PER_SEC) {
                    ts.tv_sec++;
                    ts.tv_nsec -= NSEC_PER_SEC;
                }
                missed_cycles = 0;
            }
        } else {
            missed_cycles = 0;
        }

        cycle_count++;

        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        update_motors((double) cycletime);

        if (cycle_count % config::TELEMETRY_PRINT_INTERVAL == 0) {
            print_motors();
        }

        // clock synchronisation
        if (ec_slave[0].hasdc) {
            dc_sync(ec_DCtime, cycletime, &toff);
        }

        ec_send_processdata();

        // monitor cycle time
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        long cycle_time_ns = (cycle_end.tv_sec - cycle_start.tv_sec) * NSEC_PER_SEC +
                             (cycle_end.tv_nsec - cycle_start.tv_nsec);
        if (cycle_time_ns > cycletime * 1.5) {
            printf("WARNING: Cycle time exceeded: %ld ns (expected: %ld ns)\n",
                   cycle_time_ns, (long)cycletime);
        }
    }
}

/**
 * Recover slave that has fallen out of OP state
 * @param {int} slave, {bool} report: print the action only when the slave state changed
 */
void recover_slave(int slave, bool report) {
    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
        if (report) {
            printf("ERROR: Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
        }
        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
        ec_writestate(slave);
    }
    else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
        if (report) {
            printf("WARNING: Slave %d is in SAFE_OP, changing to OPERATIONAL.\n", slave);
        }
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
    }
    else if (ec_slave[slave].state > EC_STATE_NONE) {
        if (ec_reconfig_slave(slave, config::EC_TIMEOUTMON)) {
            ec_slave[slave].islost = FALSE;
            if (report) {
                printf("MESSAGE: Slave %d reconfigured\n", slave);
            }
        }
    }
    else if (!ec_slave[slave].islost) {
        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (!ec_slave[slave].state) {
            ec_slave[slave].islost = TRUE;
            printf("ERROR: Slave %d lost\n", slave);
        }
    }
}

/**
 * Reads and prints error code 0x603F for every slave the RT thread flagged as faulted
 * Runs in the check thread so all mailbox traffic stays in one thread
 */
void report_error_codes() {
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        if (!error_code_request[slave].exchange(false, std::memory_order_relaxed)) {
            continue;
        }

        uint16_t error_code = 0;
        if (sdo_read_value(slave, INDEX_ERROR_CODE, 0x00, &error_code)) {
            printf("FAULT: Slave %d error code 0x%04X\n", slave, error_code);
        }

        // error code already cleared: fall back to the most recent entry in the error history
        if (error_code == 0) {
            uint32_t last_error = 0;
            if (sdo_read_value(slave, INDEX_ERROR_HISTORY, 0x01, &last_error)) {
                printf("FAULT: Slave %d last error history entry 0x%08X (error code 0x%04X)\n",
                       slave, last_error, last_error & 0xFFFFU);
            }
        }

        // emergency (EMCY) messages received during the mailbox read are on the SOEM error list
        while (ec_iserror()) {
            printf("       %s", ec_elist2string());
        }
    }
}

/**
 * EtherCAT bus check thread
 * Monitors working counter and slave states, publishes slave OP status to RT thread
 * attempts to recover slaves that are not operational
 */
void *ecatcheck(void *ptr) {
    (void) ptr;
    const int CHECK_PERIOD_MS = config::CHECK_PERIOD_US / 1000;
    int consecutive_errors = 0;
    // int healthy_polls = 0;
    bool monitoring = false;
    uint16_t last_state[config::MAX_MOTORS + 1] = {};

    while (1) {
        osal_usleep(config::CHECK_PERIOD_US);

        // wait for bus to reach OP
        if (!inOP) {
            monitoring = false;
            continue;
        }

        // bus just reached OP
        if (!monitoring) {
            for (int slave = 1; slave <= ec_slavecount; slave++) {
                last_state[slave] = EC_STATE_OPERATIONAL;
                slave_in_op[slave] = true;
            }
            consecutive_errors = 0;
            monitoring = true;
        }

        report_error_codes();

        // monitor working counter
        int current_wkc = wkc;
        if (current_wkc < expectedWKC) {
            consecutive_errors++;

            if (consecutive_errors == config::WKC_ERROR_THRESHOLD || 
                (consecutive_errors > config::WKC_ERROR_THRESHOLD &&
                consecutive_errors % config::WKC_REPORT_INTERVAL == 0)) {
                printf("WARNING: Working counter error (%d%d) for %d ms\n",
                current_wkc, expectedWKC, consecutive_errors * CHECK_PERIOD_MS);
            }
        }
        else {
            if (consecutive_errors >= config::WKC_ERROR_THRESHOLD) {
                printf("OK: Working counter restored (%d%d) after %d ms\n",
                current_wkc, expectedWKC, consecutive_errors * CHECK_PERIOD_MS);
            }
            consecutive_errors = 0;
        }

        // check slave states after a persistent WKC error, or while slave not in OP
        if (consecutive_errors < config::WKC_ERROR_THRESHOLD && !ec_group[currentgroup].docheckstate) {
            // bus healthy
            // if (current_wkc >= expectedWKC && ++healthy_polls % 100 == 0) {
            //     printf("OK: Bus healthy (WKC %d%d, %d slaves in OP)\n",
            //     current_wkc, expectedWKC, ec_slavecount);
            // }
            continue;
        }
        // healthy_polls = 0;

        bool any_out_before = ec_group[currentgroup].docheckstate;
        ec_group[currentgroup].docheckstate = FALSE;
        ec_readstate();

        for (int slave = 1; slave <= ec_slavecount; slave++) {
            if (ec_slave[slave].group != currentgroup) {
                continue;
            }

            uint16_t state = ec_slave[slave].state;
            bool in_op = (state == EC_STATE_OPERATIONAL);
            bool state_changed = (state != last_state[slave]);
            last_state[slave] = state;

            // publish OP status to RT thread, report transitions
            if (slave_in_op[slave] && !in_op) {
                printf("ERROR: Slave %d left OP: state 0x%02x, AL 0x%04x: %s\n",
                slave, state, ec_slave[slave].ALstatuscode,
                ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
            }
            else if (!slave_in_op[slave] && in_op) {
                printf("OK: Slave %d back in OP\n", slave);
            }
            slave_in_op[slave] = in_op;

            if (!in_op) {
                ec_group[currentgroup].docheckstate = TRUE;
                recover_slave(slave, state_changed);
            }

            if (ec_slave[slave].islost) {
                if (!ec_slave[slave].state) {
                    if (ec_recover_slave(slave, config::EC_TIMEOUTMON)) {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE: Slave %d recovered\n", slave);
                    }
                }
                else {
                    ec_slave[slave].islost = FALSE;
                    printf("MESSAGE: Slave %d found\n", slave);
                }
            }
        }

        if (any_out_before && !ec_group[currentgroup].docheckstate) {
            printf("OK: All slaves resumed operational\n");
        }
    }
}

} // namespace

bool ecat_rt_start(int cycle_time_us) {
    cycle_time_us_param = cycle_time_us;

    if (!rt_thread_create(&ecatrt_thread, "ecat_rt", config::RT_THREAD_PRIORITY,
                          config::THREAD_STACK_SIZE, ecatthread, &cycle_time_us_param)) {
        return false;
    }
    if (!rt_thread_create(&ecat_check_thread, "ecat_check", config::CHECK_THREAD_PRIORITY,
                          config::THREAD_STACK_SIZE, ecatcheck, NULL)) {
        return false;
    }
    return true;
}
