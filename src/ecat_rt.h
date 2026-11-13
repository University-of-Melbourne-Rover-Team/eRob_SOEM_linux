/**
 * EtherCAT real-time cycle thread and slave state check thread
 */

#ifndef URT_ECAT_RT_H
#define URT_ECAT_RT_H

/**
 * Starts the RT process data thread and the slave state check thread (step 6)
 * @param {int} cycle_time_us: RT cycle period in microseconds
 * @return {bool}: false if a thread could not be created
 */
bool ecat_rt_start(int cycle_time_us);

#endif
