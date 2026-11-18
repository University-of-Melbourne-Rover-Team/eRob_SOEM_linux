#include "ecat_master.h"

#include <cstdio>
#include "cia402.h"
#include "config.h"
#include "pdo.h"
#include "sdo.h"

int expectedWKC = 0;
volatile int wkc = 0;
std::atomic<bool> inOP{false};
uint8 currentgroup = 0;

static char IOmap[4096];

bool ecat_init(const char *if_name) {
    if (ec_init(if_name) <= 0) {
        printf("Error: Could not initialize EtherCAT master on %s!\n", if_name);
        printf("No socket connection on Ethernet port. Execute as root.\n");
        return false;
    }
    printf("LOG: EtherCAT master initialized successfully on %s.\n", if_name);

    if (ec_config_init(FALSE) <= 0) {
        printf("Error: Cannot find EtherCAT slaves! Nooooo!\n");
        ec_close();
        return false;
    }
    printf("LOG: %d slaves found and configured.\n", ec_slavecount);

    if (ec_slavecount > config::MAX_MOTORS) {
        printf("Error: %d slaves found, maximum supported is %d\n", ec_slavecount, config::MAX_MOTORS);
        ec_close();
        return false;
    }

    if (ec_slavecount != config::NUM_MOTORS) {
        printf("ERROR: %d slaves found. config::MOTOR_MODES lists %d motors\n", ec_slavecount, config::NUM_MOTORS);
        ec_close();
        return false;
    }

    for (int slave = 1; slave <= ec_slavecount; slave++) {
        printf("LOG: Slave %d: %s, mode %s\n", slave, ec_slave[slave].name,
            config::MOTOR_MODES[slave-1] == MODE_CSP ? "CSP" : "CSV");
    }

    return true;
}

bool ecat_request_preop() {
    // Broadcast the PRE-OP request to all slaves
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);
    if (ec_statecheck(0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE) == EC_STATE_PRE_OP) {
        printf("LOG: All slaves in PRE_OP\n");
        return true;
    }

    // Report every slave that did not reach PRE-OP
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_PRE_OP) {
            printf("ERROR: Slave %d not in PRE_OP: state 0x%02x, AL 0x%04x: %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
    }
    return false;
}

bool ecat_map_pdos() {
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        if (!pdo_map_rxpdo(slave)) {
            printf("ERROR: Slave %d RXPDO mapping failed\n", slave);
            return false;
        }
        if (!pdo_map_txpdo(slave)) {
            printf("ERROR: Slave %d TXPDO mapping failed\n", slave);
            return false;
        }
    }
    printf("LOG: PDO mapping set successfully\n");
    return true;
}

bool ecat_configure_drives() {
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        bool ret = sdo_write_value<int32_t>(slave, INDEX_MAX_VELOCITY, 0x00, config::MAX_VELOCITY) &&
                  sdo_write_value<int32_t>(slave, INDEX_MAX_ACCELERATION, 0x00, config::MAX_ACCELERATION) &&
                  sdo_write_value<int32_t>(slave, INDEX_QUICK_STOP_DECEL, 0x00, config::QUICK_STOP_DECEL);
        if (!ret) {
            printf("ERROR: Slave %d drive parameter configuration failed\n", slave);
            return false;
        }
    }
    printf("LOG: Drive parameters set successfully\n");
    return true;
}

bool ecat_config_map_and_dc() {
    ecx_context.manualstatechange = 1; // disable automatic state change
    osal_usleep(1000000);

    for (int i = 1; i <= ec_slavecount; i++) {
        printf("LOG: Slave %d: Name %s, Type %d, Address 0x%02x, State %d\n",
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
            ecx_dcsync0(&ecx_context, i, TRUE, config::SYNC0_CYCLE_NS, config::SYNC0_SHIFT_NS);
        }
    }
    osal_usleep(200000); // wait for DC configuration to take effect

    for (int i = 1; i <= ec_slavecount; i++) {
        printf("LOG: Slave %d DC status: 0x%4.4x\n", i, ec_slave[i].DCactive);
        if (ec_slave[i].hasdc && !ec_slave[i].DCactive) {
            printf("WARNING: DC not active for slave %d\n", i);
        }
    }
    return true;
}

bool ecat_request_safeop() {
    printf("LOG: Requesting SAFE_OP state...\n");
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
    printf("LOG: Successfully reached SAFE_OP state\n");

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("LOG: Calculated workcounter %d\n", expectedWKC);

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
        printf("LOG: State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL);
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

    // start slave state monitoring in the check thread once every slave in OP state
    bool all_in_op = (ec_slave[0].state == EC_STATE_OPERATIONAL);
    inOP = all_in_op;
    return all_in_op;
}

void ecat_shutdown() {
    // TODO(step 4): request INIT before ec_close(), and stop the RT threads first
    inOP = false;
    ec_close();

    printf("Request init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    printf("EtherCAT master closed.\n");
}
