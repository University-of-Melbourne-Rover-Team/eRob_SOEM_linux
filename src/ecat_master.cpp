#include "ecat_master.h"

#include <cstdio>
#include "cia402.h"
#include "config.h"
#include "pdo.h"

int expectedWKC = 0;
volatile int wkc = 0;
bool inOP = false;
uint8 currentgroup = 0;

static char IOmap[4096];

bool ecat_init(const char *if_name) {
    if (ec_init(if_name) <= 0) {
        printf("Error: Could not initialize EtherCAT master on %s!\n", if_name);
        printf("No socket connection on Ethernet port. Execute as root.\n");
        return false;
    }
    printf("EtherCAT master initialized successfully on %s.\n", if_name);

    if (ec_config_init(FALSE) <= 0) {
        printf("Error: Cannot find EtherCAT slaves! Nooooo!\n");
        ec_close();
        return false;
    }
    printf("%d slaves found and configured.\n", ec_slavecount);

    if (ec_slavecount > config::MAX_MOTORS) {
        printf("Error: %d slaves found, maximum supported is %d\n", ec_slavecount, config::MAX_MOTORS);
        ec_close();
        return false;
    }
    return true;
}

bool ecat_request_preop() {
    ec_readstate();

    // TODO(step 4): request PRE-OP once for all slaves, then check each slave
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_PRE_OP) {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            printf("Request init state for slave %d\n", i);
            ec_slave[i].state = EC_STATE_INIT;
        } else {
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            if (ec_statecheck(0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE) == EC_STATE_PRE_OP) {
                printf("State changed to EC_STATE_PRE_OP: %d\n", EC_STATE_PRE_OP);
            } else {
                printf("State EC_STATE_PRE_OP cannot be changed\n");
                return false;
            }
        }
    }
    return true;
}

bool ecat_map_pdos() {
    // TODO(step 4): check each SDO write succeeded instead of the sum being non-negative
    int retval = 0;
    for (int i = 1; i <= ec_slavecount; i++) {
        retval += pdo_map_csv_rxpdo(i);
    }
    printf("RXPDO mapping configuration result: %d\n", retval);
    if (retval < 0) {
        printf("RXPDO mapping failed\n");
        return false;
    }

    retval = 0;
    for (int i = 1; i <= ec_slavecount; i++) {
        retval += pdo_map_csv_txpdo(i);
    }
    printf("TXPDO mapping configuration result: %d\n", retval);
    if (retval < 0) {
        printf("TXPDO mapping failed\n");
        return false;
    }

    printf("PDO mapping set successfully\n");
    return true;
}

bool ecat_config_map_and_dc() {
    ecx_context.manualstatechange = 1; // disable automatic state change
    osal_usleep(1000000);

    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d: Name %s, Type %d, Address 0x%02x, State %d\n",
               i, ec_slave[i].name, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state);
    }

    // Map the configured PDOs to the IOmap
    ec_config_map(&IOmap);

    // Ensure all slaves are in PRE-OP state
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_PRE_OP) {
            printf("Slave %d not in PRE-OP state. Current state: %d, StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            return false;
        }
    }

    // Configure distributed clock, then SYNC0 once DC offsets are initialised
    printf("Configuring DC...\n");
    ec_configdc();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc) {
            ecx_dcsync0(&ecx_context, i, TRUE, config::SYNC0_CYCLE_NS, 0);
        }
    }
    osal_usleep(200000); // wait for DC configuration to take effect

    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d DC status: 0x%4.4x\n", i, ec_slave[i].DCactive);
        if (ec_slave[i].hasdc && !ec_slave[i].DCactive) {
            printf("DC not active for slave %d\n", i);
        }
    }
    return true;
}

bool ecat_request_safeop() {
    printf("Requesting SAFE_OP state...\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    osal_usleep(200000);

    int chk = 40;
    do {
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                       i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
        osal_usleep(100000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_SAFE_OP));

    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        printf("Failed to reach SAFE_OP state\n");
        return false;
    }
    printf("Successfully reached SAFE_OP state\n");

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d: State %02x, ALStatusCode %04x, Delay %d, Has DC %d, DC Active %d\n",
               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_slave[i].pdelay,
               ec_slave[i].hasdc, ec_slave[i].DCactive);
    }

    // Read back DC synchronisation configuration
    for (int i = 1; i <= ec_slavecount; i++) {
        uint16_t dcControl = 0;
        int32_t cycleTime = 0;
        int size = sizeof(dcControl);
        if (ec_SDOread(i, 0x1C32, 0x01, FALSE, &size, &dcControl, EC_TIMEOUTSAFE) > 0) {
            printf("Slave %d DC Control: 0x%04x\n", i, dcControl);
            size = sizeof(cycleTime);
            if (ec_SDOread(i, 0x1C32, 0x02, FALSE, &size, &cycleTime, EC_TIMEOUTSAFE) > 0) {
                printf("Slave %d DC Cycle Time: %d ns\n", i, cycleTime);
            }
        }
    }
    return true;
}

bool ecat_request_op() {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    if (ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE) == EC_STATE_OPERATIONAL) {
        printf("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL);
    } else {
        printf("State could not be changed to EC_STATE_OPERATIONAL\n");
        for (int i = 1; i <= ec_slavecount; i++) {
            printf("Slave %d ALstatuscode: %d\n", i, ec_slave[i].ALstatuscode);
        }
    }

    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d: Name %s, State %d, ALstatuscode 0x%04X : %s, wkc=%d/%d\n",
               i, ec_slave[i].name, ec_slave[i].state, ec_slave[i].ALstatuscode,
               ec_ALstatuscode2string(ec_slave[i].ALstatuscode), wkc, expectedWKC);
    }

    // TODO(step 3): set inOP = true here so the check thread monitors slave states
    return ec_slave[0].state == EC_STATE_OPERATIONAL;
}

void ecat_configure_drives() {
    // TODO(step 4): move these writes to PRE-OP, before the RT thread enables the drives.
    // Control word and operation mode are PDO mapped, so these SDO writes are overwritten by the PDO.
    uint8 operation_mode = config::OPERATION_MODE;
    uint16_t control_word = 0x0000;
    int32_t max_velocity = config::MAX_VELOCITY;
    int32_t max_acceleration = config::MAX_ACCELERATION;
    int32_t quick_stop_decel = config::QUICK_STOP_DECEL;

    for (int i = 1; i <= ec_slavecount; i++) {
        ec_SDOwrite(i, INDEX_CONTROL_WORD, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTSAFE);
        osal_usleep(100000);

        ec_SDOwrite(i, INDEX_OP_MODE, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);
        osal_usleep(100000);

        ec_SDOwrite(i, INDEX_MAX_VELOCITY, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, INDEX_MAX_ACCELERATION, 0x00, FALSE, sizeof(max_acceleration), &max_acceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, INDEX_QUICK_STOP_DECEL, 0x00, FALSE, sizeof(quick_stop_decel), &quick_stop_decel, EC_TIMEOUTSAFE);
        osal_usleep(100000);

        uint8 actual_mode;
        int size = sizeof(actual_mode);
        if (ec_SDOread(i, INDEX_OP_MODE_DISPLAY, 0x00, FALSE, &size, &actual_mode, EC_TIMEOUTSAFE) > 0) {
            printf("Slave %d actual operation mode: %d\n", i, actual_mode);
        }
    }
}

void ecat_shutdown() {
    // TODO(step 4): request INIT before ec_close(), and stop the RT threads first
    ec_close();

    printf("Request init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    printf("EtherCAT master closed.\n");
}
