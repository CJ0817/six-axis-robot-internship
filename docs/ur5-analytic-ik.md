# UR5 CB 解析逆解推导、分支与等价角

日期：2026-09-25。范围：与项目已核对 UR5 CB **标称几何**一致的解析推导；不是 UR5e，不含具体序列号的工厂几何标定，也不是实机验证。冻结 ABI 1.0.0 不变；本页记录推导时状态；2026-09-25后续C实现见[c-analytic-ik.md](c-analytic-ik.md)，普通分支已实现，连续族全局搜索仍待完成。本页的 Python 入口仅用于核验公式。

## 1. 模型及来源

标准 DH，列向量，m/rad，关节按基座至末端 J1～J6：

\[
A_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i),\qquad
\theta_i=s_iq_i+o_i.
\]

| i | a_i (m) | d_i (m) | alpha_i (rad) | s_i / o_i |
|---|---:|---:|---:|---|
|1|0|0.089159|π/2|1 / 0|
|2|-0.425|0|0|1 / 0|
|3|-0.39225|0|0|1 / 0|
|4|0|0.10915|π/2|1 / 0|
|5|0|0.09465|-π/2|1 / 0|
|6|0|0.0823|0|1 / 0|

参数唯一来源为 `models/ur5/kinematics.json`。依据 [UR 官方 DH 表](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/) 和 [ROS-Industrial 固定版本](https://github.com/ros-industrial/universal_robot/tree/39ad110d8f2e8f66856a201cca88aa7a7025e3eb/ur_description)，详细 URDF 来源、哈希及坐标核对见 [模型约定](ur5-model-conventions.md)。RTB 1.1.1 内置 UR5 的 d1=0.089459 与本项目差0.3 mm；基准必须用项目参数显式建模。

逻辑零位不是已标定的编码器零位。模型 base 不是 URDF base_link：两者相差 Rz(π)。输出 tool0 不是 flange。令 B=T_base_dh0、F=T_dh6_flange、G=T_flange_tool，则

\[
T_{06}=B^{-1}T_{base,tool}G^{-1}F^{-1}=\begin{bmatrix}R&p\\0&1\end{bmatrix}.
\]

本模型 B=I、FG=I，但实现必须按顺序剥离固定变换，不能据此忽略坐标标签。世界目标先转换为 base 目标。本页所有 theta 是 DH 角。

## 2. 肩分支 theta1

先移除第六轴平移：p05=p-d6 R[:,3]（这里矩阵列号为1起算）。设其 x,y 分量为 X,Y，rho=hypot(X,Y)，az=atan2(Y,X)。由 DH 链的侧向偏置得

\[
X\sin\theta_1-Y\cos\theta_1=d_4.
\]

rho<d4 时无实数肩分支；rho≥d4 时令 beta=asin(d4/rho)：

\[
\theta_1^{(0)}=az+\beta,\qquad
\theta_1^{(1)}=az+\pi-\beta.
\]

rho=d4 两支合并。本模型 d4>0，合法分支不会出现 rho=0 的 atan2 未定义情形。肩标签只表示这里公式的正负选择，不承诺对应固定的“左/右”物理命名。

## 3. 腕分支 theta5、theta6

对每个肩分支，令 h=[sin(theta1),-cos(theta1),0]，计算

\[
[u,v,c]=hR=[\sin\theta_5\cos\theta_6,
-\sin\theta_5\sin\theta_6,\cos\theta_5].
\]

设 m=hypot(u,v)。普通位形 m>epsilon_s 时，腕符号 w∈{+1,-1}：

\[
\theta_5=\operatorname{atan2}(wm,c),\qquad
\theta_6=\operatorname{atan2}(-wv,wu).
\]

这等价于 theta5=±acos(c)，但避免在 |c|≈1 时仅用 acos 损失精度，也不除以很小的 sin(theta5)。合法旋转应有 m²+c²≈1；不合法目标先按模型 pose_validation_tol 拒绝，不能靠裁剪掩盖。

## 4. 消去腕部，解肘和肩俯仰

UR5 有非零 d5，不能将后三轴当作共点球腕。对每组已知 theta1,theta5,theta6 精确消去：

\[
U=A_1^{-1}T_{06}A_6^{-1}A_5^{-1}=A_2A_3A_4.
\]

因此 U 的平移应满足

\[
x=a_2\cos\theta_2+a_3\cos(\theta_2+\theta_3),\quad
y=a_2\sin\theta_2+a_3\sin(\theta_2+\theta_3),\quad z=d_4.
\]

其旋转为 Rz(theta234)Rx(π/2)。取 x=U[1,4]、y=U[2,4]（1起算），得到

\[
c_3=\frac{x^2+y^2-a_2^2-a_3^2}{2a_2a_3},\quad
\theta_3^{(e)}=e\arccos(c_3),\quad e\in\{+1,-1\},
\]
\[
\theta_2=\operatorname{atan2}(y,x)-
\operatorname{atan2}(a_3\sin\theta_3,a_2+a_3\cos\theta_3),
\]
\[
\theta_{234}=\operatorname{atan2}(U_{21},U_{11}),\qquad
\theta_4=\theta_{234}-\theta_2-\theta_3.
\]

**a2、a3 保持负号**；不能改成正连杆长却沿用上述角公式。|c3|>1 对应当前肩/腕分支不可达，并不证明其他分支也不可达。|c3|=1 肘分支合并；theta3=±π 是同一几何肘姿态。UR5 |a2|≠|a3|，x=y=0 不满足平面两杆可达条件，不需要虚构 theta2=atan2(0,0)。

## 5. 分支及退化处理

| 条件 | 数学含义 | 处理 |
|---|---|---|
|rho>d4、m>epsilon_s、|c3|<1|普通肩×腕×肘分支|最多2×2×2=8个几何解；逐支验证，不保证总有8解|
|rho=d4|肩分支合并|对等价角规范化后去重|
|c3=±1|肘分支合并|保留一个代表，不重复计数|
|m≈0，c≈+1/-1|theta5=0/π，腕分解退化|theta6不能由 u,v 唯一确定，转入连续族处理|
|所有普通分支已严格排除|几何不可达或限位排除|区分2001与1005，不将未求完误报不可达|

腕奇异时令 theta6=phi，自由参数 phi 进入 U(phi)，继续用第4节得到 theta2/3/4(phi)。**并非任意 phi 都可行**：必须同时满足 |c3(phi)|≤1、六轴限位与回代误差；非零 d5 会使位置约束随 phi 改变。theta5=0 时姿态仅约束 theta234+theta6；theta5=π 时仅约束 theta234-theta6（模2π）；这只是姿态耦合，不代表任意增减 theta4、theta6 都保持完整末端位姿。

后续 C 实现应先求 phi 可行区间，按肘分支分段处理其端点、限位交点及距离极值，再选距 seed 最近的有效代表。直接设 phi=seed6 只是一次尝试，失败不能证明不可达。当前校验脚本仅检查6个指定 phi 的奇异见证，不实现可行区间搜索或全局最近解。

ABI `all` 只能表达最多8个离散代表，不能表达连续族。检测到有效连续族而请求穷尽全部解时应返回2003；不得从连续族随取8点称为全部解。nearest_seed 的奇异族全局选解仍待实现与专项验证；无法可靠完成时返回2003，预算耗尽返回2002。不得静默切到DLS。内部肩/腕/肘标签不写进 ABI branch_ids；该字段仍是当前排序编号。

## 6. 角度等价与限位

每个转动关节 theta_i 与 theta_i+2πk_i 的几何变换相同。恢复逻辑角 q0_i=(theta_i-o_i)/s_i（s_i=±1），其所有等价角为 q0_i+2πk_i。它们末端位姿相同，但关节绕圈状态、到达路径和距 seed 的距离不相同；不能直接作为相同设备运动指令。

令有效限位 L_i=qmin_i+margin、H_i=qmax_i-margin，则

\[
k_{min}=\left\lceil\frac{L_i-q^0_i}{2\pi}\right\rceil,\quad
k_{max}=\left\lfloor\frac{H_i-q^0_i}{2\pi}\right\rfloor.
\]

若 kmin>kmax，当前几何分支没有合法表示。否则选择使 |q0_i+2πk-seed_i| 最小的整数 k（检查最近整数两侧并限制在区间）；距离完全相等取数值较小者。六轴可分别选择，因为距离平方可加。每个几何分支只保留该代表，然后按逐轴最大绝对差≤1e-9 rad去重、字典序排序；最终按**未折返**六维欧氏距离选支，同距取字典序在前者，与冻结 ABI 一致。

本模型 J3∈[-π,π]，其余轴∈[-2π,2π]，包含端点。±π 几何等价，±2π 与0几何等价，但不能总折返到[-π,π]：例如 seed1=6.1 时，theta1=-0.2 的合法表示6.083185307…比-0.2更近。比较姿态等价可用 atan2(sin(delta),cos(delta))，选支距离不得使用该最短角差代替未折返差值。

## 7. 数值规则、错误码及实现交接

公式检查采用：肩域容差1e-12 m，acos 域容差1e-12（无量纲），epsilon_s=1e-12，去重1e-9 rad。仅越出域且不超过容差时裁剪到端点；更大越界排除分支。这些是本次数学原型常量，C实现前须用边界扫描确认，不能当作物理测量精度。回代门槛沿用位置1e-5 m、姿态角1e-4 rad。近奇异条件数很差，不保证关节角恢复到生成目标的同一组角。

生产实现步骤：校验输入/模型→剥离坐标变换→枚举普通分支或处理奇异族→等价角和收缩限位→独立 C FK 回代→排序/选解→成功时一次写输出。所有失败保持输出不变。非法参数1001，缺模型1004，限位排除1005，不支持模型1008，有完整几何排除证明2001，预算耗尽/未收敛2002，无法完成的奇异请求2003；候选异常超过8不能截断，应9000。某些分支未完成、浮点回代失败或奇异搜索未完成时不能报告2001。

确认节点：实现 C IK 前审查解析模型匹配、奇异请求策略和域容差；C IK 合入前补齐独立 C FK回代、分支完整性、限位端点、全部错误码与超时专项。推导本身不改变ABI结构或签名；KIN-03后续C验收状态以c-analytic-ik.md及指标表为准。

## 8. 可复现公式证据

```bash
# 仓库根目录；NumPy 环境，亦可使用既有 .venv-baseline/bin/python
python3 scripts/verify_ur5_analytic_derivation.py
```

报告：[results/ik-derivation/report.json](../results/ik-derivation/report.json)。本轮云端 Python3.12.14 / NumPy2.3.5；使用固定模型与先前经 C/显式RTB交叉核对的冻结目标，哈希记录于报告，没有重新生成目标。脚本不读取普通/宽初值/近奇异组的见证关节角作为求解输入。检查所有返回候选的回代，普通100/100、宽初值20/20、近奇异20/20通过；另6个指定phi奇异见证、肩不可达1例、2π等价1例通过。锚点不在140样本分母中，本轮未运行锚点集。

位置最大误差7.95e-16 m，姿态最大误差1.82e-15 rad。报告按每个目标所有候选的最大回代误差统计有效数、均值、RMSE、线性插值p95、最大值；无候选记失败、误差null，不能填0。这里的通过率仅是公式检查通过率，**不是生产 C IK 成功率、耗时或全分支完整性证明**。精确奇异测试显式提供phi，是公式代入检查，不声称解决任意奇异目标的选解。既有FFI性能未关闭项继续保留。
