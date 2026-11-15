/**
 * EtherCAT bus bring-up and shutdown
 * Call in order: init -> preop -> map PDOs -> config map/DC -> safeop -> (start RT threads) -> op -> configure drives
 */

#ifndef URT_ECAT_MASTER_H
#define URT_ECAT_MASTER_H

#include "ethercat.h"

// Bus state shared with the RT and check threads (ecat_rt.cpp)
extern int expectedWKC;     // expected working counter
extern volatile int wkc;    // last received working counter
extern bool inOP;           // true when the check thread should monitor slave states
extern uint8 currentgroup;  // EtherCAT group monitored by the check thread

/**
 * Initialises the EtherCAT master on a network interface and enumerates slaves (step 1)
 * @param {const char *} if_name: network interface string (run ip link)
 * @return {bool}: false on failure
 */
bool ecat_init(const char *if_name);

/**
 * Requests PRE-OP for all slaves (step 2)
 * @return {bool}: false on failure
 */
bool ecat_request_preop();

/**
 * Writes the CSV RXPDO and TXPDO mappings to all slaves (step 3)
 * @return {bool}: false on failure
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
 * Writes operation mode and motion limits to all drives over SDO (step 9)
 * @return {void}
 */
void ecat_configure_drives();

/**
 * Closes the EtherCAT master
 * @return {void}
 */
void ecat_shutdown();

#endif
