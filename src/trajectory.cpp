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

void csp_trajectory_reset(csp_trajectory_t *csp_trajectory, int32_t actual_position) {
    csp_trajectory->position = actual_position;
    csp_trajectory->velocity = 0.0;
    csp_trajectory->goal = actual_position;
}

void csp_trajectory_set_goal(csp_trajectory_t *csp_trajectory, int32_t goal) {
    csp_trajectory->goal = goal;
}

int32_t csp_trajectory_step(csp_trajectory_t *csp_trajectory, const csp_trajectory_limits_t *csp_limits, double dt) {
    const double dv_max = csp_limits->max_acceleration * dt;  // largest velocity change per cycle
    const double error = csp_trajectory->goal - csp_trajectory->position;

    // Finish: land exactly on the goal this cycle and stop next cycle
    const double landing_velocity = error / dt;
    if (std::fabs(landing_velocity) <= dv_max &&
        std::fabs(landing_velocity - csp_trajectory->velocity) <= dv_max) {
        csp_trajectory->position = csp_trajectory->goal;
        csp_trajectory->velocity = 0.0;
        return csp_trajectory->goal;
    }

    // desired velocity: towards the goal, capped by max velocity and by what can still stop in time
    double desired_velocity = std::fmin(csp_limits->max_velocity,
                                        braking_velocity(std::fabs(error), csp_limits->max_acceleration, dt));
    desired_velocity = std::copysign(desired_velocity, error);

    // change velocity towards desired velocity, within the acceleration limit
    const double dv = std::fmax(-dv_max, std::fmin(dv_max, desired_velocity - csp_trajectory->velocity));
    csp_trajectory->velocity += dv;
    csp_trajectory->position += csp_trajectory->velocity * dt;

    // keep the setpoint within the signed 32-bit PDO range
    csp_trajectory->position = std::fmax(static_cast<double>(INT32_MIN),
                               std::fmin(static_cast<double>(INT32_MAX), csp_trajectory->position));
    return static_cast<int32_t>(std::lround(csp_trajectory->position));
}

bool csp_trajectory_goal_reached(const csp_trajectory_t *csp_trajectory) {
    return csp_trajectory->position == csp_trajectory->goal && csp_trajectory->velocity == 0.0;
}



void csv_trajectory_reset(csv_trajectory_t *csv_trajectory) {
    csv_trajectory->goal = 0;
    csv_trajectory->velocity = 0.0;
}

void csv_trajectory_set_goal(csv_trajectory_t *csv_trajectory, int32_t goal) {
    csv_trajectory->goal = goal;
}

int32_t csv_trajectory_step(csv_trajectory_t *csv_trajectory, const csv_trajectory_limits_t *csv_limits, double dt) {
    const double dv_max = csv_limits->max_acceleration * dt;    // largest velocity change per cycle

    // keep the goal within the drive velocity limit
    const double limit = std::floor(csv_limits->max_velocity);

    // -limit <= goal <= +limit
    csv_trajectory->goal = static_cast<int32_t>(std::fmax(-limit, std::fmin(limit, (double)csv_trajectory->goal)));

    const double goal = csv_trajectory->goal;
    const double error = goal - csv_trajectory->velocity;
    if (std::fabs(error) <= dv_max) {
        csv_trajectory->velocity = goal;
    }
    else {
        csv_trajectory->velocity += std::copysign(dv_max, error);
    }

    return static_cast<int32_t>(std::lround(csv_trajectory->velocity));
}

bool csv_trajectory_goal_reached(const csv_trajectory_t *csv_trajectory) {
    return csv_trajectory->velocity == (double)csv_trajectory->goal;
}
