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
/* 笛卡尔几何路径：method1直线origin+s*u；method2圆弧
 * origin+radius*(u*cos(sweep*s)+v*sin(sweep*s))。
 * u/v为单位正交圆平面基，sweep带符号且绝对值<2pi。
 * 输出位置m、对无量纲s的一/二阶导数；count=3，s在[0,1]。
 * 0成功；1001非法输入；1008方法；失败不写输出。 */
typedef struct { int method; double origin[3], u[3], v[3], radius, sweep; } robot_cartesian_path;
int robot_path_position(const robot_cartesian_path *path, double progress,
                        double *position, double *first, double *second, size_t count);
#ifdef __cplusplus
}
#endif
#endif
