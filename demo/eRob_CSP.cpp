/* 
 * This program is an EtherCAT master implementation that initializes and configures EtherCAT slaves,
 * manages their states, and handles real-time data exchange. It includes functions for setting up 
 * PDO mappings, synchronizing time with the distributed clock, and controlling servomotors in 
 * various operational modes. The program also features multi-threading for real-time processing 
 * and monitoring of the EtherCAT network.
 */
//#include <QCoreApplication>


#include <stdio.h>
#include <string.h>
#include "ethercat.h"
#include "cia402.h"
#include "csp_command.h"
#include <iostream>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <sys/time.h>
#include <pthread.h>
#include <math.h>

#include <chrono>
#include <ctime>

#include <iostream>
#include <cstdint>

#include <sched.h>

#include <sys/socket.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cstdlib>


// Created before the cyclic thread starts; only that thread receives commands.
static int command_socket_fd = -1;

struct CommandInput {
    CspCommand latest{};
    bool available = false;
    unsigned rejected = 0;
    int receive_error = 0;
    sockaddr_un feedback_peer{};
    socklen_t feedback_peer_length = 0;
    unsigned feedback_drops = 0;
    int feedback_error = 0;
};

static int open_command_socket();
static void poll_commands(int sock, CommandInput &input);
static void publish_feedback(int sock, CommandInput &input, const CspFeedback &feedback);
static void remove_command_socket() {
    unlink(CSP_SOCKET_PATH);
}

// Global variables for EtherCAT communication
char IOmap[4096]; // I/O mapping for EtherCAT
int expectedWKC; // Expected Work Counter
boolean needlf; // Flag to indicate if a line feed is needed
volatile int wkc; // Work Counter (volatile to ensure it is updated correctly in multi-threaded context)
boolean inOP; // Flag to indicate if the system is in operational state
uint8 currentgroup = 0; // Current group for EtherCAT communication
int dorun = 0; // Flag to indicate if the thread should run
bool start_ecatthread_thread; // Flag to start the EtherCAT thread
int ctime_thread; // Cycle time for the EtherCAT thread

int64 toff, gl_delta; // Time offset and global delta for synchronization

// Function prototypes for EtherCAT thread functions
OSAL_THREAD_FUNC ecatcheck(void *ptr); // Function to check the state of EtherCAT slaves
OSAL_THREAD_FUNC_RT ecatthread(void *ptr); // Real-time EtherCAT thread function

// Thread handles for the EtherCAT threads
OSAL_THREAD_HANDLE thread1; // Handle for the EtherCAT check thread
OSAL_THREAD_HANDLE thread2; // Handle for the real-time EtherCAT thread

// Function to synchronize time with the EtherCAT distributed clock
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
// Function to add nanoseconds to a timespec structure
void add_timespec(struct timespec *ts, int64 addtime);

// Define constants for stack size and timing
#define stack64k (64 * 1024) // Stack size for threads
#define NSEC_PER_SEC 1000000000   // Number of nanoseconds in one second
#define EC_TIMEOUTMON 5000        // Timeout for monitoring in microseconds
#define MAX_VELOCITY 30000        // Reduced maximum velocity (from 200000 to 30000)
#define MAX_ACCELERATION 50000    // Reduced maximum acceleration (from 500000 to 50000)

// Conversion units for the servomotor
float Cnt_to_deg = 0.000686645; // Conversion factor from counts to degrees
int8_t SLAVE_ID; // Slave ID for EtherCAT communication

// Structure for RXPDO (Control data sent to slave)
typedef struct {
    uint16_t controlword;      // 0x6040:0, 16 bits
    int32_t target_position;   // 0x607A:0, 32 bits
    uint8_t mode_of_operation; // 0x6060:0, 8 bits
    uint8_t padding;           // 8 bits padding for alignment
} __attribute__((__packed__)) rxpdo_t;

// Structure for TXPDO (Status data received from slave)
typedef struct {
    uint16_t statusword;      // 0x6041:0, 16 bits
    int32_t actual_position;  // 0x6064:0, 32 bits
    int32_t actual_velocity;  // 0x606C:0, 32 bits
    int16_t actual_torque;    // 0x6077:0, 16 bits
} __attribute__((__packed__)) txpdo_t;

// The cyclic thread never waits for status readers or terminal output.
struct CycleStatus {
    uint16_t statusword = 0;
    int32_t actual_position = 0;
    int32_t target_position = 0;
    int32_t destination_position = 0;
    int cycle_number = 0;
    int32_t actual_velocity = 0;
    int16_t actual_torque = 0;
    int workcounter = 0;
    long cycle_ns = 0;
    unsigned overruns = 0;
    unsigned sleep_errors = 0;
    unsigned rejected_commands = 0;
    int command_error = 0;
    int feedback_error = 0;
};
static CycleStatus cycle_status[EC_MAXSLAVE];
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;

// 在文件开头，其他宏定义之后添加
#undef MAX_VELOCITY  // Ensure there are no naming conflicts
#undef MAX_ACCELERATION

// 在全局变量声明区域添加
struct MotionPlanner {
    bool initialized = false;
    int32_t target_position = 0;
    double current_position = 0.0; // Keep fractional counts between cycles.
    double current_velocity = 0.0;

    static constexpr double MAX_VELOCITY = 50000.0;     // counts/s
    static constexpr double MAX_ACCELERATION = 50000.0; // counts/s^2
    static constexpr double BRAKE_DECEL = 5000.0;       // counts/s^2
};

constexpr double MotionPlanner::MAX_VELOCITY;
constexpr double MotionPlanner::MAX_ACCELERATION;
constexpr double MotionPlanner::BRAKE_DECEL;

int32_t plan_trajectory(MotionPlanner* planner, int32_t actual_position,
                        double cycle_seconds);

//##################################################################################################
// Function: Set the CPU affinity for a thread
void set_thread_affinity(pthread_t thread, int cpu_core) {
    cpu_set_t cpuset; // CPU set to specify which CPUs the thread can run on
    CPU_ZERO(&cpuset); // Clear the CPU set
    CPU_SET(cpu_core, &cpuset); // Add the specified CPU core to the set

    // Set the thread's CPU affinity
    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        printf("Unable to set CPU affinity for thread %d\n", cpu_core); // Error message if setting fails
    } else {
        printf("Thread successfully bound to CPU %d\n", cpu_core); // Confirmation message if successful
    }
}

//##################################################################################################
// Function prototype for the EtherCAT test function
int erob_test();

uint16_t data_R;

int erob_test() {
    int rdl; // Variable to hold read data length
    SLAVE_ID = 1; // Set the slave ID to 1
    int i, j, oloop, iloop, chk; // Loop control variables

    // 1. Call ec_config_init() to move from INIT to PRE-OP state.
    printf("__________STEP 1___________________\n");
    // Initialize EtherCAT master on the specified network interface
    if (ec_init("enp89s0") <= 0) {
        printf("Error: Could not initialize EtherCAT master!\n");
        printf("No socket connection on Ethernet port. Execute as root.\n");
        printf("___________________________________________\n");
        return -1; // Return error if initialization fails
    }
    printf("EtherCAT master initialized successfully.\n");
    printf("___________________________________________\n");

    // Search for EtherCAT slaves on the network
    if (ec_config_init(FALSE) <= 0) {
        printf("Error: Cannot find EtherCAT slaves!\n");
        printf("___________________________________________\n");
        ec_close(); // Close the EtherCAT connection
        return -1; // Return error if no slaves are found
    }
    if (ec_slavecount != CSP_MOTOR_COUNT) {
        fprintf(stderr, "CSP commands require exactly %d slaves; found %d.\n",
                CSP_MOTOR_COUNT, ec_slavecount);
        ec_close();
        return -1;
    }
    printf("%d slaves found and configured.\n", ec_slavecount); // Print the number of slaves found
    printf("___________________________________________\n");

    // 2. Change to pre-operational state to configure the PDO registers
    printf("__________STEP 2___________________\n");

    // Check if the slave is ready to map
    ec_readstate(); // Read the state of the slaves

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // If the slave is not in PRE-OP state
            // Print the current state and status code of the slave
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            printf("\nRequest init state for slave %d\n", i); // Request to change the state to INIT
            ec_slave[i].state = EC_STATE_INIT; // Set the slave state to INIT
            printf("___________________________________________\n");
        } else { // If the slave is in PRE-OP state
            ec_slave[0].state = EC_STATE_PRE_OP; // Set the first slave to PRE-OP state
            /* Request EC_STATE_PRE_OP state for all slaves */
            ec_writestate(0); // Write the state change to the slave
            /* Wait for all slaves to reach the PRE-OP state */
            if ((ec_statecheck(0, EC_STATE_PRE_OP,  3 * EC_TIMEOUTSTATE)) == EC_STATE_PRE_OP) {
                printf("State changed to EC_STATE_PRE_OP: %d \n", EC_STATE_PRE_OP);
                printf("___________________________________________\n");
            } else {
                printf("State EC_STATE_PRE_OP cannot be changed in step 2\n");
                return -1; // Return error if state change fails
            }
        }
    }

//##################################################################################################
    //3.- Map RXPOD
    printf("__________STEP 3___________________\n");

    // Clear RXPDO mapping
    int retval = 0; // Variable to hold the return value of SDO write operations
    uint16 map_1c12; // Variable to hold the mapping for PDO
    uint8 zero_map = 0; // Variable to clear the PDO mapping
    uint32 map_object; // Variable to hold the mapping object
    uint16 clear_val = 0x0000; // Value to clear the mapping

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        // 1. First, disable PDO
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);
        
        // 2. Configure new PDO mapping
        // Control Word
        map_object = 0x60400010;  // 0x6040:0 Control Word, 16 bits
        retval += ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Target Position
        map_object = 0x607A0020;  // 0x607A:0 Target Position, 32 bits
        retval += ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Mode of Operation
        map_object = 0x60600008;  // 0x6060:0 Mode of Operation, 8 bits
        retval += ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Padding (8 bits)
        map_object = 0x00000008;  // 8 bits padding
        retval += ec_SDOwrite(i, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Set number of mapped objects
        uint8 map_count = 4;
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        
        // 4. Configure RXPDO allocation
        clear_val = 0x0000; // Clear the mapping
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_1c12 = 0x1600; // Set the mapping to the new PDO
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        map_1c12 = 0x0001; // Set the mapping index
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    }

    printf("PDO mapping configuration result: %d\n", retval);
    if (retval < 0) {
        printf("PDO mapping failed\n");
        return -1;
    }

    printf("RXPOD Mapping set correctly.\n");
    printf("___________________________________________\n");

    //........................................................................................
    // Map TXPOD
    retval = 0;
    uint16 map_1c13;
    for(int i = 1; i <= ec_slavecount; i++) {
        // First, clear the TXPDO mapping
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Configure TXPDO mapping entries
        // Status Word (0x6041:0, 16 bits)
        map_object = 0x60410010;
        retval += ec_SDOwrite(i, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Position (0x6064:0, 32 bits)
        map_object = 0x60640020;
        retval += ec_SDOwrite(i, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Velocity (0x606C:0, 32 bits)
        map_object = 0x606C0020;
        retval += ec_SDOwrite(i, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Torque (0x6077:0, 16 bits)
        map_object = 0x60770010;
        retval += ec_SDOwrite(i, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Set the number of mapped objects (4 objects)
        uint8 map_count = 4;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

        // Configure TXPDO assignment
        // First, clear the assignment
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Assign TXPDO to 0x1A00
        map_1c13 = 0x1A00;
        retval += ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

        // Set the number of assigned PDOs (1 PDO)
        map_1c13 = 0x0001;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    }

    printf("Slave %d TXPDO mapping configuration result: %d\n", SLAVE_ID, retval);

    if (retval < 0) {
        printf("TXPDO Mapping failed\n");
        printf("___________________________________________\n");
        return -1;
    }

    printf("TXPDO Mapping set successfully\n");
    printf("___________________________________________\n");

   //##################################################################################################

    //4.- Set ecx_context.manualstatechange = 1. Map PDOs for all slaves by calling ec_config_map().
   printf("__________STEP 4___________________\n");

   ecx_context.manualstatechange = 1; //Disable automatic state change
   osal_usleep(1e6); //Sleep for 1 second

    uint8 WA = 0; //Variable for write access
    uint8 my_RA = 0; //Variable for read access
    uint32 TIME_RA; //Variable for time read access

    // Print the information of the slaves found
    for (int i = 1; i <= ec_slavecount; i++) {
       // (void)ecx_FPWR(ecx_context.port, i, ECT_REG_DCSYNCACT, sizeof(WA), &WA, 5 * EC_TIMEOUTRET);
        printf("Name: %s\n", ec_slave[i].name); //Print the name of the slave
        printf("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_INIT);
        printf("___________________________________________\n");
    }

    // Map the configured PDOs to the IOmap
    ec_config_map(&IOmap);

    printf("__________STEP 5___________________\n");

    // Ensure all slaves are in PRE-OP state
    for(int i = 1; i <= ec_slavecount; i++) {
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // Check if the slave is not in PRE-OP state
            printf("Slave %d not in PRE-OP state. Current state: %d\n", i, ec_slave[i].state);
            return -1; // Return error if any slave is not in PRE-OP state
        }
    }

    // Configure Distributed Clock (DC)
    ec_configdc(); // Set up the distributed clock for synchronization

    // Configure SYNC0 after the DC clocks and offsets have been initialized.
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc) {
            ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
        }
    }

    // Request to switch to SAFE-OP state
    ec_slave[0].state = EC_STATE_SAFE_OP; // Set the first slave to SAFE-OP state
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP) {
        printf("Successfully changed to SAFE_OP state\n"); // Confirm successful state change
    } else {
        printf("Failed to change to SAFE_OP state\n");
        return -1; // Return error if state change fails
    }

    // Calculate the expected Work Counter (WKC)
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC; // Calculate expected WKC based on outputs and inputs
    printf("Calculated workcounter %d\n", expectedWKC);

    // Read and display basic status information of the slaves
    ec_readstate(); // Read the state of all slaves
    for(int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d\n", i);
        printf("  State: %02x\n", ec_slave[i].state); // Print the state of the slave
        printf("  ALStatusCode: %04x\n", ec_slave[i].ALstatuscode); // Print the AL status code
        printf("  Delay: %d\n", ec_slave[i].pdelay); // Print the delay of the slave
        printf("  Has DC: %d\n", ec_slave[i].hasdc); // Check if the slave supports Distributed Clock
        printf("  DC Active: %d\n", ec_slave[i].DCactive); // Check if DC is active for the slave
        printf("  DC supported: %d\n", ec_slave[i].hasdc); // Print if DC is supported
    }

    // Read DC synchronization configuration using the correct parameters
    for(int i = 1; i <= ec_slavecount; i++) {
        uint16_t dcControl = 0; // Variable to hold DC control configuration
        int32_t cycleTime = 0; // Variable to hold cycle time
        int32_t shiftTime = 0; // Variable to hold shift time
        int size; // Variable to hold size for reading

        // Read DC synchronization configuration, adding the correct size parameter
        size = sizeof(dcControl);
        if (ec_SDOread(i, 0x1C32, 0x01, FALSE, &size, &dcControl, EC_TIMEOUTSAFE) > 0) {
            printf("Slave %d DC Configuration:\n", i);
            printf("  DC Control: 0x%04x\n", dcControl); // Print the DC control configuration
            
            size = sizeof(cycleTime);
            if (ec_SDOread(i, 0x1C32, 0x02, FALSE, &size, &cycleTime, EC_TIMEOUTSAFE) > 0) {
                printf("  Cycle Time: %d ns\n", cycleTime); // Print the cycle time
            }

        }
    }

    printf("__________STEP 6___________________\n");

    // Start the EtherCAT thread for real-time processing
    start_ecatthread_thread = TRUE; // Flag to indicate that the EtherCAT thread should start
    osal_thread_create_rt(&thread1, stack64k * 2, (void *)&ecatthread, (void *)&ctime_thread); // Create the real-time EtherCAT thread
    // set_thread_affinity(*thread1, 4); // Optional: Set CPU affinity for the thread
    osal_thread_create(&thread2, stack64k * 2, (void *)&ecatcheck, NULL); // Create the EtherCAT check thread
    // set_thread_affinity(*thread2, 5); // Optional: Set CPU affinity for the thread
    printf("___________________________________________\n");

    my_RA = 0; // Reset read access variable


    // 8. Transition to OP state
    printf("__________STEP 8___________________\n");

    // Send process data to the slaves
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive process data and store the Work Counter

    // Set the first slave to operational state
    ec_slave[0].state = EC_STATE_OPERATIONAL; // Change the state of the first slave to OP
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition to complete
    if ((ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE)) == EC_STATE_OPERATIONAL) {
        printf("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL); // Confirm successful state change
        printf("___________________________________________\n");
    } else {
        printf("State could not be changed to EC_STATE_OPERATIONAL\n"); // Error message if state change fails
        for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
            printf("ALstatuscode: %d\n", ecx_context.slavelist[cnt].ALstatuscode); // Print AL status codes for each slave
        }
    }

    // Read and display the state of all slaves
    ec_readstate(); // Read the state of all slaves
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_OPERATIONAL); // Print slave information
        printf("Name: %s\n", ec_slave[i].name); // Print the name of the slave
        printf("___________________________________________\n");
    }

    // 9. Configure servomotor and mode operation
    printf("__________STEP 9___________________\n");

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("Operational state reached for all slaves.\n");
        

        uint8 operation_mode = 8;
        uint16_t Control_Word = 128;

        for (int i = 1; i <= ec_slavecount; i++) {
            ec_SDOwrite(i, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);

        }
  // Status and command errors are printed outside the cyclic thread.
        unsigned reported_rejections = 0;
        int reported_command_error = 0;
        int reported_feedback_error = 0;
        while(1) {
            osal_usleep(100000);
            CycleStatus snapshot[EC_MAXSLAVE];
            pthread_mutex_lock(&status_mutex);
            memcpy(snapshot, cycle_status, (ec_slavecount + 1) * sizeof(CycleStatus));
            pthread_mutex_unlock(&status_mutex);
            if (snapshot[1].rejected_commands != reported_rejections) {
                reported_rejections = snapshot[1].rejected_commands;
                fprintf(stderr, "Ignored malformed CSP commands: %u (expected six int32 positions).\n",
                        reported_rejections);
            }
            if (snapshot[1].feedback_error != reported_feedback_error) {
                reported_feedback_error = snapshot[1].feedback_error;
                if (reported_feedback_error != 0)
                    fprintf(stderr, "CSP feedback send: %s\n", strerror(reported_feedback_error));
            }
            if (snapshot[1].command_error != reported_command_error) {
                reported_command_error = snapshot[1].command_error;
                fprintf(stderr, "CSP command input disabled: %s. Last destinations retained.\n",
                        strerror(reported_command_error));
            }
            for (int slave = 1; slave <= ec_slavecount; slave++) {
                const CycleStatus &status = snapshot[slave];
                printf("Slave %d: cycle=%d, SW=0x%04X, pos=%d, target=%d, goal=%d, vel=%d\n",
                       slave, status.cycle_number, status.statusword, status.actual_position,
                       status.target_position, status.destination_position, status.actual_velocity);
            }
        }
    }

    osal_usleep(1e6);

    ec_close();

     printf("\nRequest init state for all slaves\n");
     ec_slave[0].state = EC_STATE_INIT;
     /* request INIT state for all slaves */
     ec_writestate(0);

    printf("EtherCAT master closed.\n");

    return 0;
}

/* 
 * PI calculation to synchronize Linux time with the Distributed Clock (DC) time.
 * This function calculates the offset time needed to align the Linux time with the DC time.
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0; // Integral term for PI controller
    int64 delta; // Variable to hold the difference between reference time and cycle time
    delta = (reftime) % cycletime; // Calculate the delta time
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime; // Adjust delta if it's greater than half the cycle time
    }
    if (delta > 0) {
        integral++; // Increment integral if delta is positive
    }
    if (delta < 0) {
        integral--; // Decrement integral if delta is negative
    }
    *offsettime = -(delta / 100) - (integral / 20); // Calculate the offset time
    gl_delta = delta; // Update global delta variable
}

/* 
 * Add nanoseconds to a timespec structure.
 * This function updates the timespec structure by adding a specified amount of time.
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec; // Variables to hold seconds and nanoseconds

    nsec = addtime % NSEC_PER_SEC; // Calculate nanoseconds to add
    sec = (addtime - nsec) / NSEC_PER_SEC; // Calculate seconds to add
    ts->tv_sec += sec; // Update seconds in timespec
    ts->tv_nsec += nsec; // Update nanoseconds in timespec
    if (ts->tv_nsec >= NSEC_PER_SEC) { // If nanoseconds exceed 1 second
        nsec = ts->tv_nsec % NSEC_PER_SEC; // Adjust nanoseconds
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC; // Increment seconds
        ts->tv_nsec = nsec; // Set adjusted nanoseconds
    }
}

/* 
 * EtherCAT check thread function
 * This function monitors the state of the EtherCAT slaves and attempts to recover 
 * any slaves that are not in the operational state.
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr) {
    int slave; // Variable to hold the current slave index
    (void)ptr; // Not used
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    while (1) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                printf("\n");
            }
            
            // Increase the consecutive error count
            if (wkc < expectedWKC) {
                consecutive_errors++;
                printf("WARNING: Working counter error (%d/%d), consecutive errors: %d\n", 
                       wkc, expectedWKC, consecutive_errors);
            } else {
                consecutive_errors = 0;
            }

            // If the consecutive errors exceed the threshold, attempt reinitialization
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                printf("ERROR: Too many consecutive errors, attempting recovery...\n");
                ec_group[currentgroup].docheckstate = TRUE;
                // Reset the error count
                consecutive_errors = 0;
            }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
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
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
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
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
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

/* 
 * RT EtherCAT thread function
 * This function handles the real-time processing of EtherCAT data. 
 * It sends and receives process data in a loop, synchronizing with the 
 * distributed clock if available, and ensuring timely execution based on 
 * the specified cycle time.
 */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr) {
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    int missed_cycles = 0;
    const int MAX_MISSED_CYCLES = 10;
    struct timespec cycle_start, cycle_end;
    long cycle_time_ns;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int *)ptr * 1000;

    toff = 0;
    dorun = 0;
    
    // Initialize PDO data
    rxpdo_t rxpdo[EC_MAXSLAVE]{};
    txpdo_t txpdo[EC_MAXSLAVE]{};
    MotionPlanner motion_planners[EC_MAXSLAVE];
    bool hold_position_latched[EC_MAXSLAVE]{};
    CommandInput command_input;
    CspFeedback feedback{};
    memcpy(feedback.magic, "CSF1", sizeof(feedback.magic));
    // configure RXPDO data on startup
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        rxpdo[slave].controlword = CW_FAULT_RESET_CMD;
        rxpdo[slave].target_position = 0; 
        rxpdo[slave].mode_of_operation = MODE_CSP;
        rxpdo[slave].padding = 0;
    }
    
    // Send initial process data
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        memcpy(ec_slave[slave].outputs, &rxpdo[slave], sizeof(rxpdo_t));
    }
    ec_send_processdata();

    int step = 0;
    unsigned cycle_overruns = 0;
    unsigned sleep_errors = 0;

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);
        
        add_timespec(&ts, cycletime + toff);
        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft) != 0) {
            // If sleep is interrupted, record the error
            missed_cycles++;
            ++sleep_errors;
            if (missed_cycles >= MAX_MISSED_CYCLES) {
                // Reset the counter
                missed_cycles = 0;
                // Resynchronize the clock
                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;
                if (ts.tv_nsec >= NSEC_PER_SEC) {
                    ts.tv_sec++;
                    ts.tv_nsec -= NSEC_PER_SEC;
                }
            }
        } else {
            missed_cycles = 0;
        }
        
        dorun++;

        if (start_ecatthread_thread) {
            // Receive process data
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            // Consume queued commands even if this cycle's PDO feedback is bad.
            // Only this thread owns the complete destination array.
            poll_commands(command_socket_fd, command_input);

            if (wkc >= expectedWKC) {
                // Keep each slave's feedback separate.
                for (int slave = 1; slave <= ec_slavecount; slave++) {
                    memcpy(&txpdo[slave], ec_slave[slave].inputs, sizeof(txpdo_t));
                }

                // Preserve CSP's existing stage thresholds. Like CSV, each
                // stage waits for every slave to report its expected state.
                uint16_t controlword;
                uint16_t expected_state;
                if (step <= 400) {
                    controlword = CW_FAULT_RESET_CMD;
                    expected_state = SW_STATE_SWITCH_ON_DISABLED;
                } else if (step <= 600) {
                    controlword = CW_SHUTDOWN_CMD;
                    expected_state = SW_STATE_READY_TO_SWITCH_ON;
                } else if (step <= 800) {
                    controlword = CW_SWITCH_ON_CMD;
                    expected_state = SW_STATE_SWITCHED_ON;
                } else {
                    controlword = CW_ENABLE_OP_CMD;
                    expected_state = SW_STATE_OPERATION_ENABLED;
                }

                bool next_state_ready = true;
                for (int slave = 1; slave <= ec_slavecount; slave++) {
                    if (cia402_decode_state(txpdo[slave].statusword) != expected_state)
                        next_state_ready = false;
                }

                // Array entry zero is slave 1; all six destinations change together.
                const bool command_available = command_input.available;
                const bool motion_ready = step > 1000 && next_state_ready;
                feedback.cycle = static_cast<uint32_t>(dorun);
                feedback.flags = motion_ready && ec_slave[0].state == EC_STATE_OPERATIONAL
                    ? CSP_FEEDBACK_READY : 0;
                for (int slave = 1; slave <= ec_slavecount; slave++) {
                    feedback.positions[slave - 1] = txpdo[slave].actual_position;
                    feedback.statuswords[slave - 1] = txpdo[slave].statusword;
                    rxpdo[slave].controlword = controlword;
                    rxpdo[slave].mode_of_operation = MODE_CSP;

                    const uint16_t drive_state = cia402_decode_state(txpdo[slave].statusword);
                    const bool enabling_or_enabled = controlword == CW_ENABLE_OP_CMD &&
                        (drive_state == SW_STATE_SWITCHED_ON ||
                         drive_state == SW_STATE_OPERATION_ENABLED);
                    if (!enabling_or_enabled) {
                        // Align with feedback while disabled/faulted. Re-latch
                        // before enabling so an old target is not applied on recovery.
                        rxpdo[slave].target_position = txpdo[slave].actual_position;
                        hold_position_latched[slave] = false;
                    } else if (!hold_position_latched[slave]) {
                        // Capture once, before sending Enable Operation. Copying
                        // actual_position every cycle would let the target drift.
                        rxpdo[slave].target_position = txpdo[slave].actual_position;
                        hold_position_latched[slave] = true;
                    }

                    if (motion_ready && command_available) {
                        motion_planners[slave].target_position =
                            command_input.latest.positions[slave - 1];
                        rxpdo[slave].target_position = plan_trajectory(
                            &motion_planners[slave], txpdo[slave].actual_position,
                            static_cast<double>(cycletime) / NSEC_PER_SEC);
                    } else {
                        // Keep the latched position (or last commanded setpoint)
                        // while enabled, including when another slave is not ready.
                        motion_planners[slave].initialized = false;
                    }
                    memcpy(ec_slave[slave].outputs, &rxpdo[slave], sizeof(rxpdo_t));
                }

                if (step < 1200 && next_state_ready) {
                    step++;
                }
            }

            // Clock synchronization
            if (ec_slave[0].hasdc) {
                ec_sync(ec_DCtime, cycletime, &toff);
            }

            // EtherCAT transmission takes priority over optional viewer feedback.
            ec_send_processdata();
            if (wkc >= expectedWKC && dorun % 10 == 0)
                publish_feedback(command_socket_fd, command_input, feedback);
        }

        // Monitor cycle time
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        cycle_time_ns = (cycle_end.tv_sec - cycle_start.tv_sec) * NSEC_PER_SEC +
                       (cycle_end.tv_nsec - cycle_start.tv_nsec);
        
        if (cycle_time_ns > cycletime * 1.5)
            ++cycle_overruns;

        // Skip publication if the main thread is copying the previous sample.
        if (pthread_mutex_trylock(&status_mutex) == 0) {
            for (int slave = 1; slave <= ec_slavecount; slave++) {
                CycleStatus &status = cycle_status[slave];
                status.statusword = txpdo[slave].statusword;
                status.actual_position = txpdo[slave].actual_position;
                status.target_position = rxpdo[slave].target_position;
                status.destination_position = command_input.available
                    ? command_input.latest.positions[slave - 1] : rxpdo[slave].target_position;
                status.cycle_number = dorun;
                status.actual_velocity = txpdo[slave].actual_velocity;
                status.actual_torque = txpdo[slave].actual_torque;
                status.workcounter = wkc;
                status.cycle_ns = cycle_time_ns;
                status.overruns = cycle_overruns;
                status.sleep_errors = sleep_errors;
                status.rejected_commands = command_input.rejected;
                status.command_error = command_input.receive_error;
                status.feedback_error = command_input.feedback_error;
            }
            pthread_mutex_unlock(&status_mutex);
        }
    }
}

int correct_count = 0;
int incorrect_count = 0;
int test_count_sum = 100;
int test_count = 0;
float correct_rate = 0;

// Advance one CSP setpoint using the real PDO period and the latest destination.
// This function never waits for a move to finish.
int32_t plan_trajectory(MotionPlanner* planner, int32_t actual_position,
                        double cycle_seconds) {
    if (!planner->initialized) {
        planner->current_position = actual_position;
        planner->current_velocity = 0.0;
        planner->initialized = true;
    }

    const double error = static_cast<double>(planner->target_position) -
                         planner->current_position;
    if (fabs(error) < 0.5 &&
        fabs(planner->current_velocity) <= planner->MAX_ACCELERATION * cycle_seconds) {
        planner->current_position = planner->target_position;
        planner->current_velocity = 0.0;
        return planner->target_position;
    }

    const double allowed_speed = sqrt(2.0 * planner->BRAKE_DECEL * fabs(error));
    const double desired_velocity =
        copysign(fmin(planner->MAX_VELOCITY, allowed_speed), error);
    const double velocity_error = desired_velocity - planner->current_velocity;
    const double max_velocity_change = planner->MAX_ACCELERATION * cycle_seconds;
    const double previous_velocity = planner->current_velocity;
    planner->current_velocity +=
        fmax(-max_velocity_change, fmin(max_velocity_change, velocity_error));
    planner->current_position +=
        0.5 * (previous_velocity + planner->current_velocity) * cycle_seconds;

    // Keep rounding and conversion inside the signed 32-bit PDO range.
    planner->current_position = fmax(static_cast<double>(INT32_MIN),
                                    fmin(static_cast<double>(INT32_MAX),
                                         planner->current_position));
    return static_cast<int32_t>(llround(planner->current_position));
}

// Socket setup runs before the cyclic thread, never on its timing path.
static int open_command_socket() {
    const int sock = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (sock < 0) {
        perror("CSP socket");
        return -1;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, CSP_SOCKET_PATH, sizeof(addr.sun_path) - 1);
    // Do not unlink another running master's socket.
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        const int bind_error = errno;
        fprintf(stderr, "CSP socket bind: %s\n", strerror(bind_error));
        if (bind_error == EADDRINUSE)
            fprintf(stderr, "If no CSP master is running, remove stale %s and retry.\n",
                    CSP_SOCKET_PATH);
        close(sock);
        return -1;
    }
    // Allow the invoking desktop user to command a master launched with sudo,
    // without making the motor command socket writable by every local user.
    const char *sudo_uid = std::getenv("SUDO_UID");
    if (geteuid() == 0 && sudo_uid != nullptr) {
        char *end = nullptr;
        errno = 0;
        const unsigned long owner = std::strtoul(sudo_uid, &end, 10);
        if (errno != 0 || end == sudo_uid || *end != '\0' ||
            owner >= static_cast<unsigned long>(static_cast<uid_t>(-1))) {
            fprintf(stderr, "Invalid SUDO_UID for CSP socket ownership\n");
            close(sock);
            unlink(CSP_SOCKET_PATH);
            return -1;
        }
        if (chown(CSP_SOCKET_PATH, static_cast<uid_t>(owner), static_cast<gid_t>(-1)) < 0) {
            perror("CSP socket chown");
            close(sock);
            unlink(CSP_SOCKET_PATH);
            return -1;
        }
    }
    if (chmod(CSP_SOCKET_PATH, S_IRUSR | S_IWUSR) < 0) {
        perror("CSP socket chmod");
        close(sock);
        unlink(CSP_SOCKET_PATH);
        return -1;
    }
    printf("CSP command socket: %s (six positions, slave 1 through 6)\n", CSP_SOCKET_PATH);
    return sock;
}

static void poll_commands(int sock, CommandInput &input) {
    if (input.receive_error != 0)
        return;

    // Bound syscall work even if a sender continuously fills the queue.
    // Under overload the remaining messages are consumed on subsequent ticks.
    constexpr unsigned MAX_COMMANDS_PER_TICK = 32;
    for (unsigned n = 0; n < MAX_COMMANDS_PER_TICK; ++n) {
        CspCommand candidate{};
        sockaddr_un sender{};
        socklen_t sender_length = sizeof(sender);
        const ssize_t received = recvfrom(sock, &candidate, sizeof(candidate),
            MSG_DONTWAIT | MSG_TRUNC, reinterpret_cast<sockaddr*>(&sender), &sender_length);
        if (received < 0) {
            const int error = errno;
            if (error == EAGAIN || error == EWOULDBLOCK)
                return; // Empty queue: keep advancing toward the last destinations.
            if (error == EINTR)
                continue; // Retries also count against this tick's limit.
            input.receive_error = error;
            return; // Main thread reports the error; never print in this loop.
        }
        if (received == sizeof(CSP_SUBSCRIBE) &&
            memcmp(&candidate, CSP_SUBSCRIBE, sizeof(CSP_SUBSCRIBE)) == 0) {
            // Only a bound client has an address to receive feedback. A new
            // subscriber replaces the previous viewer; CLI commands don't steal it.
            if (sender_length > sizeof(sender.sun_family) && sender_length <= sizeof(sender)) {
                input.feedback_peer = sender;
                input.feedback_peer_length = sender_length;
                input.feedback_error = 0;
            } else {
                ++input.rejected;
            }
            continue;
        }
        if (received != static_cast<ssize_t>(sizeof(candidate))) {
            ++input.rejected; // Truncated/short/empty datagrams are consumed, not applied.
            continue;
        }
        input.latest = candidate;
        input.available = true;
        // recv consumes the datagram. Later valid messages replace this one.
    }
}

static void publish_feedback(int sock, CommandInput &input, const CspFeedback &feedback) {
    if (input.feedback_peer_length == 0)
        return;
    const ssize_t sent = sendto(sock, &feedback, sizeof(feedback), MSG_DONTWAIT,
        reinterpret_cast<const sockaddr*>(&input.feedback_peer), input.feedback_peer_length);
    if (sent == static_cast<ssize_t>(sizeof(feedback))) {
        input.feedback_error = 0;
        return;
    }
    const int error = sent < 0 ? errno : EIO;
    ++input.feedback_drops;
    // A slow viewer never blocks the PDO cycle. Drop this sample and try the
    // next fresh one; no telemetry backlog is kept in the real-time thread.
    if (error == EAGAIN || error == EWOULDBLOCK || error == EINTR)
        return;
    input.feedback_error = error;
    input.feedback_peer_length = 0; // Client heartbeat can subscribe again.
}

int main(int argc, char **argv) {
    needlf = FALSE;
    inOP = FALSE;
    start_ecatthread_thread = FALSE;
    dorun = 0;
    ctime_thread = 1000; // 1ms cycle time

    // Configuration and status printing use normal scheduling.
    // osal_thread_create_rt() requests FIFO priority 40 for the cyclic thread.
    struct sched_param param{};
    if (sched_setscheduler(0, SCHED_OTHER, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    // Lock memory to prevent paging
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }

    // Set CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity");
        return EXIT_FAILURE;
    }

    command_socket_fd = open_command_socket();
    if (command_socket_fd < 0)
        return EXIT_FAILURE;
    // Process exit closes the fd; unlink only the pathname we successfully bound.
    std::atexit(remove_command_socket);

    printf("Running on CPU core 3\n");
    const int result = erob_test();
    printf("End program\n");

    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
