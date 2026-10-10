#ifndef ROBOT_PLANNING_H
#define ROBOT_PLANNING_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* 独立规划接口 v0.1；不改变 robot_contract 的冻结运动学 ABI。
 * SI 单位，J1..J6。method: 1三次、2五次、3梯形（可退化三角形）。 */
typedef struct {
    int method;
    double duration_s, minimum_duration_s, ramp_s, cruise_s;
    double q_start[6], delta[6];
} robot_joint_plan;
/* 静止到静止的六轴同步规划。
 * start/goal/lower/upper/vmax/amax: 6个double，单位rad、rad/s、rad/s²。
 * count必须6；requested_s=0自动定时，否则>0；stretch=0严格/1延长。
 * sample_s>0、max_s>=sample_s。成功写out；失败不改out。
 * 返回0；1001非法值/指针/长度，1005位置越限，1006非法时间，
 * 3002严格时长违反速度/加速度，3001超过时长预算，1008不支持method。 */
int robot_plan_joint(int method, const double *start, const double *goal,
                     const double *lower, const double *upper,
                     const double *vmax, const double *amax, size_t count,
                     double requested_s, int stretch, double sample_s,
                     double max_s, robot_joint_plan *out);
/* 解析求值。time_s必须在[0,duration]，不允许外推。
 * q/qd/qdd均6元素输出；成功0，失败不写输出。切换点qdd右连续，
 * 末端qdd采用段内左极限，三次/梯形不伪造零端加速度。
 * 返回1001非法指针/长度/plan，1006时间越界/非有限。 */
int robot_joint_evaluate(const robot_joint_plan *plan, double time_s,
                         double *q, double *qd, double *qdd, size_t count);
#ifdef __cplusplus
}
#endif
#endif
