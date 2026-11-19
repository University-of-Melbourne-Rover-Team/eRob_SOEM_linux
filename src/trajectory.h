/**
 * Trajectory planner for CSP position setpoints and CSV target velocities
 */

#ifndef URT_TRAJECTORY_H
#define URT_TRAJECTORY_H

#include <cstdint>

/**
 * CSP structs
 */

typedef struct {
    double max_velocity;        // counts/s
    double max_acceleration;    // counts/s^2, used for both accel and decel
} csp_trajectory_limits_t;

typedef struct {
    double position;    // current setpoint (counts)
    double velocity;    // current setpoint velocity (counts/s)
    int32_t goal;       // position the setpoint is moving towards (counts)
} csp_trajectory_t;


/**
 * CSV structs
 */

 typedef struct {
    double max_velocity;        // counts/s
    double max_acceleration;    // counts/s^2, used for both accel and decel
} csv_trajectory_limits_t;

typedef struct {
    double velocity;    // current setpoint velocity (counts/s)
    int32_t goal;       // velocity the setpoint is moving towards (counts/s)
} csv_trajectory_t;


/**
 * CSP Trajectory Planning Functions
 */

/**
 * Resets the trajectory at standstill
 * @param {csp_trajectory_t *} csp_trajectory, {int32_t} actual_position
 * @return {void}
 */
void csp_trajectory_reset(csp_trajectory_t *csp_trajectory, int32_t actual_position);

/**
 * Sets a new goal position, the current position and velocity are kept so motion stays smooth
 * @param {csp_trajectory_t *} trajectory, {int32_t} goal: goal position
 * @return {void}
 */
void csp_trajectory_set_goal(csp_trajectory_t *csp_trajectory, int32_t goal);

/**
 * Advances the setpoint by one cycle towards the goal
 * @param {csp_trajectory_t *} csp_trajectory, {const trajectory_limits_t *} csp_limits, {double} dt: cycle time (s)
 * @return {int32_t}: position setpoint for this cycle (counts)
 */
int32_t csp_trajectory_step(csp_trajectory_t *csp_trajectory, const csp_trajectory_limits_t *csp_limits, double dt);

/**
 * @param {const csp_trajectory_t *} trajectory
 * @return {bool}: true if the setpoint is stopped on the goal
 */
bool csp_trajectory_goal_reached(const csp_trajectory_t *csp_trajectory);


/**
 * CSV Trajectory Planning Functions
 */

/**
 * Resets the velocity to 0
 * @param {csv_trajectory_t *} csv_trajectory
 * @return {void}
 */
void csv_trajectory_reset(csv_trajectory_t *csv_trajectory);

/**
 * Sets a new goal velocity
 * @param {csv_trajectory_t *} csv_trajectory, {int32_t} goal: goal velocity
 * @return {void}
 */
void csv_trajectory_set_goal(csv_trajectory_t *csv_trajectory, int32_t goal);

/**
 * Advances the velocity setpoint by one cycle towards goal
 * @param {csv_trajectory_t *} csv_trajectory, {const csv_trajectory_limits_t *} csv_limits, {double} dt: cycle time (s)
 * @return {int32_t}: position setpoint for this cycle (counts)
 */
int32_t csv_trajectory_step(csv_trajectory_t *csv_trajectory, const csv_trajectory_limits_t *csv_limits, double dt);

/**
 * @param {const csv_trajectory_t *} csv_trajectory
 * @return {bool}: true if the velocity reached the target
 */
bool csv_trajectory_goal_reached(const csv_trajectory_t *csv_trajectory);

#endif
