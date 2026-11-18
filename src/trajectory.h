/**
 * Trajectory planner for CSP position setpoints
 */

#ifndef URT_TRAJECTORY_H
#define URT_TRAJECTORY_H

#include <cstdint>

typedef struct {
    double max_velocity;        // counts/s
    double max_acceleration;    // counts/s^2, used for both accel and decel
} trajectory_limits_t;

typedef struct {
    double position;    // current setpoint (counts)
    double velocity;    // current setpoint velocity (counts/s)
    int32_t goal;       // position the setpoint is moving towards (counts)
} trajectory_t;

/**
 * Resets the trajectory at standstill
 * @param {trajectory_t *} trajectory, {int32_t} actual_position
 * @return {void}
 */
void trajectory_reset(trajectory_t *trajectory, int32_t actual_position);

/**
 * Sets a new goal position, the current position and velocity are kept so motion stays smooth
 * @param {trajectory_t *} trajectory, {int32_t} goal: goal position
 * @return {void}
 */
void trajectory_set_goal(trajectory_t *trajectory, int32_t goal);

/**
 * Advances the setpoint by one cycle towards the goal
 * @param {trajectory_t *} trajectory, {const trajectory_limits_t *} limits, {double} dt: cycle time (s)
 * @return {int32_t}: position setpoint for this cycle (counts)
 */
int32_t trajectory_step(trajectory_t *trajectory, const trajectory_limits_t *limits, double dt);

/**
 * @param {const trajectory_t *} trajectory
 * @return {bool}: true if the setpoint is stopped on the goal
 */
bool trajectory_goal_reached(const trajectory_t *trajectory);

#endif
