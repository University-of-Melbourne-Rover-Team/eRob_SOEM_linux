#include "trajectory.h"

#include <cmath>
#include <cstdint>

namespace {

/**
 * Fastest allowable velocity the setpoint can move
 * @param {double} distance: distance to goal, {double} accel (counts/s^2), {double} dt (s)
 * @return {double}: braking velocity
 */
double braking_velocity(double distance, double accel, double dt) {
    const double dv = accel * dt;   // velocity change per cycle
    const double k = std::sqrt(0.25 + 2.0 * distance / (dv * dt)) - 0.5;
    return k * dv;
}

} // namespace

void trajectory_reset(trajectory_t *trajectory, int32_t actual_position) {
    trajectory->position = actual_position;
    trajectory->velocity = 0.0;
    trajectory->goal = actual_position;
}

void trajectory_set_goal(trajectory_t *trajectory, int32_t goal) {
    trajectory->goal = goal;
}

int32_t trajectory_step(trajectory_t *trajectory, const trajectory_limits_t *limits, double dt) {
    const double dv_max = limits->max_acceleration * dt;  // largest velocity change per cycle
    const double error = trajectory->goal - trajectory->position;

    // Finish: land exactly on the goal this cycle and stop next cycle
    const double landing_velocity = error / dt;
    if (std::fabs(landing_velocity) <= dv_max &&
        std::fabs(landing_velocity - trajectory->velocity) <= dv_max) {
        trajectory->position = trajectory->goal;
        trajectory->velocity = 0.0;
        return trajectory->goal;
    }

    // desired velocity: towards the goal, capped by max velocity and by what can still stop in time
    double desired_velocity = std::fmin(limits->max_velocity,
                                        braking_velocity(std::fabs(error), limits->max_acceleration, dt));
    desired_velocity = std::copysign(desired_velocity, error);

    // change velocity towards desired velocity, within the acceleration limit
    const double dv = std::fmax(-dv_max, std::fmin(dv_max, desired_velocity - trajectory->velocity));
    trajectory->velocity += dv;
    trajectory->position += trajectory->velocity * dt;

    // keep the setpoint within the signed 32-bit PDO range
    trajectory->position = std::fmax(static_cast<double>(INT32_MIN),
                               std::fmin(static_cast<double>(INT32_MAX), trajectory->position));
    return static_cast<int32_t>(std::lround(trajectory->position));
}

bool trajectory_goal_reached(const trajectory_t *trajectory) {
    return trajectory->position == trajectory->goal && trajectory->velocity == 0.0;
}
