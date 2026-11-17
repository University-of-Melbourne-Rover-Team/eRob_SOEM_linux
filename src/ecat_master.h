/**
 * EtherCAT bus bring-up and shutdown
 * Call in order: init -> preop -> map PDOs -> configure drives -> config map/DC -> safeop -> (start RT threads) -> op
 */

#ifndef URT_ECAT_MASTER_H
#define URT_ECAT_MASTER_H

#include "ethercat.h"

#include <atomic>

// Bus state shared with the RT and check threads (ecat_rt.cpp)
extern int expectedWKC;         // expected working counter
extern volatile int wkc;        // last received working counter
extern std::atomic<bool> inOP;  // true when the check thread should monitor slave states
extern uint8 currentgroup;      // EtherCAT group monitored by the check thread

/**
 * Initialises the EtherCAT master on a network interface and enumerates slaves (step 1)
 * @param {const char *} if_name: network interface string (run ip link)
 * @return {bool}: false on failure
 */
bool ecat_init(const char *if_name);

/**
 * Requests PRE-OP for all slaves (step 2)
 * @return {bool}: false if any slave not in pre-op
 */
bool ecat_request_preop();

/**
 * Writes the RXPDO and TXPDO mappings to all slaves
 * @return {bool}: false if any SDO write failed
 */
bool ecat_map_pdos();

/**
 * Maps PDOs into the IOmap and configures distributed clocks / SYNC0 (steps 4-5)
 * @return {bool}: false on failure
 */
bool ecat_config_map_and_dc();

/**
 * Requests SAFE-OP for all slaves and calculates the expected working counter (step 5)
 * @return {bool}: false on failure
 */
bool ecat_request_safeop();

/**
 * Requests OP for all slaves (step 8). The RT thread must already be running.
 * @return {bool}: true if all slaves reached OP
 */
bool ecat_request_op();

/**
 * Writes motion limits to all drives over SDO
 * Slaves must be in PRE-OP
 * @return {bool} true if drive parameters were set successfully
 */
bool ecat_configure_drives();

/**
 * Closes the EtherCAT master
 * @return {void}
 */
void ecat_shutdown();

#endif
