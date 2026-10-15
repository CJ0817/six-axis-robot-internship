#include "robot_control.h"
#include <math.h>
#include <stddef.h>
#define PID_READY UINT32_C(0x50494431)

/* Validate explicit config; returns a shared error code without mutation. */
static int validate(const robot_pid_config *c) {
    if (!c) return ROBOT_INVALID_ARGUMENT;
    if (!isfinite(c->dt_s) || c->dt_s <= 0.0) return ROBOT_INVALID_TIME;
    if (!isfinite(c->kp) || c->kp < 0.0 ||
        !isfinite(c->ki) || c->ki < 0.0 ||
        !isfinite(c->kd) || c->kd < 0.0 ||
        !isfinite(c->integral_min) || !isfinite(c->integral_max) ||
        !isfinite(c->output_min) || !isfinite(c->output_max) ||
        c->integral_min > 0.0 || c->integral_max < 0.0 ||
        c->integral_min > c->integral_max ||
        c->output_min > 0.0 || c->output_max < 0.0 ||
        c->output_min >= c->output_max) return ROBOT_INVALID_ARGUMENT;
    return ROBOT_OK;
}

/* Clamp a finite value to inclusive limits, returning the bounded value. */
static double clamp(double x, double lo, double hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

int robot_pid_reset(const robot_pid_config *c, robot_pid_state *s) {
    robot_pid_state fresh = {0};
    int code = validate(c);
    if (code) return code;
    if (!s) return ROBOT_INVALID_ARGUMENT;
    fresh.config = *c;
    fresh.initialized = PID_READY;
    *s = fresh;
    return ROBOT_OK;
}

int robot_pid_step(robot_pid_state *s, double reference,
                   double actual, robot_pid_result *r) {
    robot_pid_state next;
    robot_pid_result out = {0};
    const robot_pid_config *c;
    double e, increment, tentative, di, dp, dd, raw;
    if (!s || !r || !isfinite(reference) || !isfinite(actual))
        return ROBOT_INVALID_ARGUMENT;
    if (s->initialized != PID_READY || validate(&s->config) ||
        !isfinite(s->error_1) || !isfinite(s->error_2) ||
        !isfinite(s->integral) || !isfinite(s->raw_output) ||
        !isfinite(s->output)) return ROBOT_INVALID_CONTROL_STATE;
    c = &s->config;
    if (s->integral < c->integral_min || s->integral > c->integral_max ||
        s->output < c->output_min || s->output > c->output_max)
        return ROBOT_INVALID_CONTROL_STATE;
    e = reference - actual;
    increment = c->ki * c->dt_s * e;
    tentative = s->integral + increment;
    dp = c->kp * (e - s->error_1);
    dd = (c->kd / c->dt_s) * (e - 2.0*s->error_1 + s->error_2);
    if (!isfinite(e) || !isfinite(increment) || !isfinite(tentative) ||
        !isfinite(dp) || !isfinite(dd)) return ROBOT_INVALID_ARGUMENT;
    out.integral = clamp(tentative, c->integral_min, c->integral_max);
    out.integral_limited = out.integral != tentative;
    di = out.integral - s->integral;
    raw = s->raw_output + dp + di + dd;
    if (!isfinite(raw)) return ROBOT_INVALID_ARGUMENT;
    /* Conditional integration prevents accumulating further into saturation.
     * Retain the raw accumulator so clipping does not erase the P/D terms. */
    if ((raw > c->output_max && di > 0.0) ||
        (raw < c->output_min && di < 0.0)) {
        out.integral = s->integral;
        out.integral_limited = 1;
        raw = s->raw_output + dp + dd;
    }
    if (!isfinite(raw)) return ROBOT_INVALID_ARGUMENT;
    out.error = e;
    out.raw_output = raw;
    out.output = clamp(raw, c->output_min, c->output_max);
    out.output_limited = out.output != raw;
    next = *s;
    next.error_2 = s->error_1;
    next.error_1 = e;
    next.integral = out.integral;
    next.raw_output = raw;
    next.output = out.output;
    *s = next;
    *r = out;
    return ROBOT_OK;
}
