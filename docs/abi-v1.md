# C 算法库 ABI 1.0.0（冻结，2026-09-19）

规范头文件为 `include/robot_kinematics.h` 与 `include/robot_errors.h`，机器可读快照为 `src/common/abi-v1.json`。既有 FK、矩阵函数和 robot_copy_joints 保持原签名与布局；本次只增加ABI查询和版本化IK入口。逆解算法尚未实现，不代表本规范已通过逆解精度验收。

## 二进制平台与稳定性

基线为Linux x86_64、SysV C调用约定、LP64、小端；double为IEEE754 binary64，int/uint32_t为32位，size_t/指针为64位。自然8字节结构对齐，禁止pragma pack。C++使用extern "C"。库名librobot_contract.so，头文件包含完整参数声明。

`uint32_t robot_abi_version(void)` 返回0x00010000；高16位主版本、低16位次版本。补丁修复不得改变签名、参数顺序、字段偏移、含义、错误码或内存所有权。新能力以新符号/新结构体版本增加；不兼容变化须新主版本库与迁移方案，保留旧库。不得在现有结构末尾追加字段或重用枚举值。审查变更时检查ABI快照，CI同时验证C静态断言、ctypes偏移、导出符号及错误码。

## 正逆运动学签名（参数顺序固定）

```c
int robot_forward(const robot_fk_model *model,
                  const double *q_rad, size_t count,
                  double *T_base_tool);
int robot_inverse_v1(const robot_fk_model *model,
                     const double *T_base_tool, size_t pose_count,
                     const double *q_seed_rad, size_t seed_count,
                     const robot_ik_options_v1 *options,
                     robot_ik_result_v1 *result);
```

FK：count必须为6，输出空间必须可写16个double。IK：pose_count必须为16，seed_count必须为6，result至少504字节。数组连续、按行存储：矩阵元素T[i,j]在索引4*i+j，候选关节candidates[k][j]在6*k+j。几何使用列向量；长度m、角度rad；J1～J6；目标和输出均为已核对模型的base→tool0。C结构不含字符串，调用方须完成单位、轴序、坐标标签校验；Python适配器负责这些检查。

调用者拥有模型、输入、输出；库不分配返回堆对象、不保存指针、不修改输入，无全局求解状态。同步调用期间内存必须有效且不被其他线程修改。矩阵/FK内部暂存后写出，支持已声明的输入输出别名；IK禁止result与输入区间重叠。任意悬空指针、虚假分配容量不属于可恢复错误，不能声称C能探测实际缓冲区大小。

成功码0时输出有效；任何失败保持可用输出缓冲区逐字节不变。C低层不返回JSON；Python上层必须将非0返回映射为data=null，不得复用上次输出。异常/崩溃/硬超时由隔离进程执行器处理，不能依赖C错误码捕获非法内存访问。

## 固定模型布局

robot_fk_model大小704字节、对齐8。用于纯FK/IK几何，不含动力学或启动控制所需字段。

| 字段 | 类型/元素数 | 字节偏移 |
| --- | --- | --- |
| a_m / alpha_rad / d_m / theta_offset_rad | 各double[6] | 0 / 48 / 96 / 144 |
| joint_sign | int[6]，各±1 | 192 |
| T_base_dh0 / T_dh6_flange / T_flange_tool | 各double[16] | 216 / 344 / 472 |
| q_min_rad / q_max_rad | 各double[6] | 600 / 648 |
| pose_validation_tol | double，0<值≤1e-6 | 696 |

数值必须有限；每轴min<max；固定变换须是合法刚体矩阵。关节限位包括端点，不自动折返角度。零偏与符号关系、变换顺序沿用正运动学推导，不变更既有行为。

## IK选项布局与约束

robot_ik_options_v1大小72字节、对齐8，所有字段必须显式填写。

| 字段 | 类型 | 偏移 | 约束 |
| --- | --- | --- | --- |
| method | uint32_t | 0 | 1=analytic_ur5；2=DLS |
| branch_policy | uint32_t | 4 | 0=nearest_seed；1=all（仅解析法） |
| position_tol_m | double | 8 | >0 |
| orientation_tol_rad | double | 16 | 0<值≤π |
| joint_margin_rad | double | 24 | ≥0；收缩限位后每轴区间非空 |
| timeout_s | double | 32 | >0；测试基线0.2 s |
| max_iterations | uint32_t | 40 | DLS>0；解析法填0 |
| line_search_max_steps | uint32_t | 44 | DLS>0；解析法填0 |
| characteristic_length_m | double | 48 | DLS>0；解析法填0 |
| damping | double | 56 | DLS>0；解析法填0 |
| max_step_rad | double | 64 | DLS>0；解析法填0 |

analytic_ur5仅支持已约定的UR5几何结构；不受支持的模型结构返回1008，不猜测套用其他机器人的解析式。DLS不支持all分支请求，返回1008；未知方法/策略返回1008；非有限选项或不满足字段约束返回1001。target须满足模型pose_validation_tol，seed须在收缩后的限位内，seed越界返回1005。普通几何奇异位形不必失败：若能返回有效有限代表解则允许成功，不能枚举的连续解族不谎称为全部解。

## IK结果布局

robot_ik_result_v1大小504字节、对齐8。

| 字段 | 类型 | 偏移 | 成功时含义 |
| --- | --- | --- | --- |
| q_rad | double[6] | 0 | 所选关节解 |
| candidates_rad | double[8][6] | 48 | 前solution_count行有效，剩余行填0 |
| branch_ids | uint32_t[8] | 432 | 有效候选在本次输出排序中的编号，0起；未用槽为UINT32_MAX |
| solution_count | uint32_t | 464 | 1～8；nearest_seed只返回1个，all返回全部离散代表候选 |
| selected_index | uint32_t | 468 | <solution_count，q_rad与该行完全一致 |
| position_error_m / orientation_error_rad | double | 472 / 480 | 所选解FK回代误差，分别满足给定阈值 |
| elapsed_s | double | 488 | C单调墙钟测得接口总耗时，有限且≥0 |
| iterations | uint32_t | 496 | DLS实际迭代数；解析法为0 |
| reserved | uint32_t | 500 | 必须为0，不允许后续直接改变含义 |

每个解析几何分支在限位内允许有多个2π等价表示时，选择离seed欧氏距离最小的表示（相同距离按q的字典序）。数值上相同的候选按最大逐轴绝对差≤1e-9 rad去重，随后按(q1,…,q6)字典序排序；选解按未折返的六维欧氏距离，距离精确相同时取排序靠前者。branch_ids只表示本次返回的排序编号，不是跨目标稳定的肩/肘/腕标签。每个候选都必须通过FK回代和限位验证。不得直接截断第9个非等价候选并宣称all完整，算法应遵守UR5离散分支上限；否则9000。

## 统一错误码及占位状态

编号唯一来源为contract.json，冻结对应C枚举见robot_errors.h，完整编号快照见abi-v1.json；禁止另建逆解专用冲突编号。常用：0成功、1001参数错误、1002单位、1003轴序、1004模型缺失、1005关节超限、1007坐标、1008未实现、2001有证明的不可达、2002未收敛或超时、2003无法完成请求的奇异性、9000内部错误。其余规划/控制/I/O编号原样保留。

未来IK校验先检查必要指针及长度（1001；单独model为空为1004），再按有限性/模型/矩阵/选项约束、seed限位、算法请求顺序处理；DLS不收敛不能直接返回2001。失败结果不可执行，结果结构保持原字节；不通过同一个失败输出返回诊断数据，父进程另记耗时和返回码。

**当前robot_inverse_v1是明确占位实现：对所有调用直接返回1008，不解引用输入、不写输出。** 参数错误码的完整校验语义在实现算法时启用；本次没有逆解成功结果，KIN-03仍待完成。冻结符号和布局先供上层集成，不以占位代码替代算法验收。

## 验证与使用

编译命令沿用forward-kinematics.md；`python -m unittest discover -s tests -p test_abi_v1.py -v` 验证ABI。C编译期逐字段静态断言拒绝偏移或尺寸变化。对已编译的旧FK调用方，既有robot_forward和robot_fk_model仍保持二进制布局；原最小ABI探针与C++/Python旧测试继续执行。
