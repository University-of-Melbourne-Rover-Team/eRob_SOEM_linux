/**
 * URT EtherCAT maindevice
 * Usage: sudo ./main [if_name]
 */

#include <cstdio>
#include <cstdlib>
#include <iterator>

#include "ethercat.h"
#include "config.h"
#include "ecat_master.h"
#include "ecat_rt.h"
#include "rt_utils.h"

/**
 * Brings the EtherCAT bus to OP and runs the motors
 * @param {const char *} if_name: network interface string (run ip link)
 * @return {int}: program status code
 */
static int run(const char *if_name) {
    // NOTE: have to wait 8 seconds after power before sending commands to ZeroErr motors.
    if (!ecat_init(if_name)) return -1;
    if (!ecat_request_preop()) return -1;
    if (!ecat_map_pdos()) return -1;
    if (!ecat_config_map_and_dc()) return -1;
    if (!ecat_request_safeop()) return -1;

    if (!ecat_rt_start(config::CYCLE_TIME_US)) return -1;
    printf("Wait one second before requesting OP...\n");
    osal_usleep(1000000);

    if (ecat_request_op()) {
        printf("Operational state reached for all slaves.\n");
        ecat_configure_drives();

        // TODO(step 4): replace with signal-driven shutdown
        while (1) {
            osal_usleep(100000);
        }
    }

    osal_usleep(1000000);
    ecat_shutdown();
    return 0;
}

int main(int argc, char **argv) {
    // set network interface name from command line
    // use default if no command line input
    const char *if_name = (argc > 1) ? argv[1] : config::DEFAULT_IF_NAME;

    if (!rt_setup_process(config::PROCESS_PRIORITY, config::PROCESS_CPU_CORES,
                          std::size(config::PROCESS_CPU_CORES))) {
        return EXIT_FAILURE;
    }

    run(if_name);
    printf("End program\n");
    return EXIT_SUCCESS;
}
