#include "ecat_rt.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <pthread.h>

#include "ethercat.h"
#include "cia402.h"
#include "config.h"
#include "ecat_master.h"
#include "rt_utils.h"

namespace {

pthread_t ecatrt_thread;
pthread_t ecat_check_thread;
int cycle_time_us_param;

// Motor state, index 0 unused so motors[i] matches ec_slave[i]
cia402_motor_t motors[config::MAX_MOTORS + 1];

/**
 * Updates connection state, CiA 402 state machine and targets for every motor (one RT cycle)
 */
void update_motors() {
    /**
     * Check the connection status of each slave on the bus using working counter
     * Each slave expected to increment working counter 3 times each
     * i.e. 10 slaves --> wkc == 30
     * TODO(step 3): WKC is a bus total, it cannot identify which slave dropped
     */
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        bool slave_connected = (wkc >= (slave * 3));
        // motor just disconnected (falling edge)
        if (motors[slave].ec_connected && !slave_connected) {
            cia402_lost_motor(&motors[slave]);
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

        // set target velocities
        if (motors[slave].operation_enabled) {
            motors[slave].rxpdo.target_velocity = config::DEMO_TARGET_VELOCITY;
        } else {
            motors[slave].rxpdo.target_velocity = 0;
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
        printf("Motor %d: SW=0x%04x, CW=0x%04x, pos=%d, vel=%d, target_vel=%d, mode=%d, fault=%d, op_en=%d, ec_connected=%d\n", slave,
               motors[slave].txpdo.statusword,
               motors[slave].rxpdo.controlword,
               motors[slave].txpdo.actual_position,
               motors[slave].txpdo.actual_velocity,
               motors[slave].rxpdo.target_velocity,
               motors[slave].rxpdo.mode_of_operation,
               motors[slave].faulted,
               motors[slave].operation_enabled,
               motors[slave].ec_connected);
    }
    printf("\n");
}

/**
 * RT EtherCAT thread
 * Sends and receives process data every cycle, synchronised to the distributed clock
 */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr) {
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
        cia402_init_motor(&motors[slave], config::OPERATION_MODE);
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
        update_motors();

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
 * EtherCAT check thread
 * Monitors slave states and attempts to recover slaves that are not operational
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr) {
    (void)ptr;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    while (1) {
        // TODO(step 3): inOP is never set, so this block does not run yet
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (wkc < expectedWKC) {
                consecutive_errors++;
                printf("WARNING: Working counter error (%d/%d), consecutive errors: %d\n",
                       wkc, expectedWKC, consecutive_errors);
            } else {
                consecutive_errors = 0;
            }

            // TODO(step 3): docheckstate is cleared straight after being set below
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                printf("ERROR: Too many consecutive errors, attempting recovery...\n");
                ec_group[currentgroup].docheckstate = TRUE;
                consecutive_errors = 0;
            }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("ERROR: Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf("WARNING: Slave %d is in SAFE_OP, changing to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, config::EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE: Slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state) {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR: Slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (!ec_slave[slave].state) {
                        if (ec_recover_slave(slave, config::EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE: Slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE: Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate) {
                printf("OK: All slaves resumed OPERATIONAL.\n");
            }
        }
        osal_usleep(10000);
    }
}

} // namespace

bool ecat_rt_start(int cycle_time_us) {
    cycle_time_us_param = cycle_time_us;

    // TODO(step 3): RT thread runs at FIFO 40 while the check thread inherits the process priority (99)
    if (!osal_thread_create_rt(&ecatrt_thread, config::THREAD_STACK_SIZE, (void *)&ecatthread, &cycle_time_us_param)) {
        printf("Error: Could not create RT EtherCAT thread\n");
        return false;
    }
    if (!osal_thread_create(&ecat_check_thread, config::THREAD_STACK_SIZE, (void *)&ecatcheck, NULL)) {
        printf("Error: Could not create EtherCAT check thread\n");
        return false;
    }
    return true;
}
