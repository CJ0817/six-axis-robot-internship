#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <stdint.h>
#include "robot_errors.h"
#if defined(_WIN32)
#define ROBOT_CONTROL_API __declspec(dllexport)
#else
#define ROBOT_CONTROL_API
#endif
#ifdef __cplusplus
extern "C" {
#endif
/* Independent single-joint PID ABI v0.1; does not change kinematics ABI v1. */
typedef struct {
    double dt_s, kp, ki, kd;
    double integral_min, integral_max, output_min, output_max;
} robot_pid_config;
typedef struct {
    robot_pid_config config;
    double error_1, error_2, integral, raw_output, output;
    uint32_t initialized;
} robot_pid_state;
typedef struct {
    double error, integral, raw_output, output;
    uint32_t integral_limited, output_limited;
} robot_pid_result;
/* Initialize/reset all history. c: finite gains >=0, dt>0, bounds include 0.
 * s: caller-owned state. Returns 0, 1001 or 1006; failure leaves s unchanged.
 * No allocation or I/O; state is not a device enable/fault state machine. */
ROBOT_CONTROL_API int robot_pid_reset(const robot_pid_config *c, robot_pid_state *s);
/* Execute one fixed-period sample. reference/actual: rad, no angle wrapping.
 * Returns diagnostics in r and advances s only on success. Invalid input=1001,
 * invalid state=4001. Saturation is normal operation, reported in flags.
 * Output units depend on explicit config/plant (rad/s or N m).
 * s and r must point to separate caller-owned objects. */
ROBOT_CONTROL_API int robot_pid_step(robot_pid_state *s, double reference,
                                    double actual, robot_pid_result *r);
#ifdef __cplusplus
}
#endif
#endif
