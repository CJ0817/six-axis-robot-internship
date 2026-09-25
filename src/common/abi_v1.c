#include "robot_kinematics.h"
#include <float.h>
#include <limits.h>
#if !defined(__linux__) || !defined(__x86_64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "ABI v1 currently supports Linux x86_64 little-endian only"
#endif
_Static_assert(CHAR_BIT==8 && sizeof(double)==8 && DBL_MANT_DIG==53 && sizeof(int)==4 && sizeof(size_t)==8, "ABI v1 requires LP64 binary64");
_Static_assert(sizeof(robot_fk_model)==704 && _Alignof(robot_fk_model)==8, "robot_fk_model layout changed");
_Static_assert(offsetof(robot_fk_model,a_m)==0, "robot_fk_model.a_m offset changed");
_Static_assert(offsetof(robot_fk_model,alpha_rad)==48, "robot_fk_model.alpha_rad offset changed");
_Static_assert(offsetof(robot_fk_model,d_m)==96, "robot_fk_model.d_m offset changed");
_Static_assert(offsetof(robot_fk_model,theta_offset_rad)==144, "robot_fk_model.theta_offset_rad offset changed");
_Static_assert(offsetof(robot_fk_model,joint_sign)==192, "robot_fk_model.joint_sign offset changed");
_Static_assert(offsetof(robot_fk_model,T_base_dh0)==216, "robot_fk_model.T_base_dh0 offset changed");
_Static_assert(offsetof(robot_fk_model,T_dh6_flange)==344, "robot_fk_model.T_dh6_flange offset changed");
_Static_assert(offsetof(robot_fk_model,T_flange_tool)==472, "robot_fk_model.T_flange_tool offset changed");
_Static_assert(offsetof(robot_fk_model,q_min_rad)==600, "robot_fk_model.q_min_rad offset changed");
_Static_assert(offsetof(robot_fk_model,q_max_rad)==648, "robot_fk_model.q_max_rad offset changed");
_Static_assert(offsetof(robot_fk_model,pose_validation_tol)==696, "robot_fk_model.pose_validation_tol offset changed");
_Static_assert(sizeof(robot_ik_options_v1)==72 && _Alignof(robot_ik_options_v1)==8, "robot_ik_options_v1 layout changed");
_Static_assert(offsetof(robot_ik_options_v1,method)==0, "robot_ik_options_v1.method offset changed");
_Static_assert(offsetof(robot_ik_options_v1,branch_policy)==4, "robot_ik_options_v1.branch_policy offset changed");
_Static_assert(offsetof(robot_ik_options_v1,position_tol_m)==8, "robot_ik_options_v1.position_tol_m offset changed");
_Static_assert(offsetof(robot_ik_options_v1,orientation_tol_rad)==16, "robot_ik_options_v1.orientation_tol_rad offset changed");
_Static_assert(offsetof(robot_ik_options_v1,joint_margin_rad)==24, "robot_ik_options_v1.joint_margin_rad offset changed");
_Static_assert(offsetof(robot_ik_options_v1,timeout_s)==32, "robot_ik_options_v1.timeout_s offset changed");
_Static_assert(offsetof(robot_ik_options_v1,max_iterations)==40, "robot_ik_options_v1.max_iterations offset changed");
_Static_assert(offsetof(robot_ik_options_v1,line_search_max_steps)==44, "robot_ik_options_v1.line_search_max_steps offset changed");
_Static_assert(offsetof(robot_ik_options_v1,characteristic_length_m)==48, "robot_ik_options_v1.characteristic_length_m offset changed");
_Static_assert(offsetof(robot_ik_options_v1,damping)==56, "robot_ik_options_v1.damping offset changed");
_Static_assert(offsetof(robot_ik_options_v1,max_step_rad)==64, "robot_ik_options_v1.max_step_rad offset changed");
_Static_assert(sizeof(robot_ik_result_v1)==504 && _Alignof(robot_ik_result_v1)==8, "robot_ik_result_v1 layout changed");
_Static_assert(offsetof(robot_ik_result_v1,q_rad)==0, "robot_ik_result_v1.q_rad offset changed");
_Static_assert(offsetof(robot_ik_result_v1,candidates_rad)==48, "robot_ik_result_v1.candidates_rad offset changed");
_Static_assert(offsetof(robot_ik_result_v1,branch_ids)==432, "robot_ik_result_v1.branch_ids offset changed");
_Static_assert(offsetof(robot_ik_result_v1,solution_count)==464, "robot_ik_result_v1.solution_count offset changed");
_Static_assert(offsetof(robot_ik_result_v1,selected_index)==468, "robot_ik_result_v1.selected_index offset changed");
_Static_assert(offsetof(robot_ik_result_v1,position_error_m)==472, "robot_ik_result_v1.position_error_m offset changed");
_Static_assert(offsetof(robot_ik_result_v1,orientation_error_rad)==480, "robot_ik_result_v1.orientation_error_rad offset changed");
_Static_assert(offsetof(robot_ik_result_v1,elapsed_s)==488, "robot_ik_result_v1.elapsed_s offset changed");
_Static_assert(offsetof(robot_ik_result_v1,iterations)==496, "robot_ik_result_v1.iterations offset changed");
_Static_assert(offsetof(robot_ik_result_v1,reserved)==500, "robot_ik_result_v1.reserved offset changed");
uint32_t robot_abi_version(void) { return ROBOT_ABI_VERSION_V1; }
