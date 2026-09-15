# UR5 模型来源与适配

模型来自 [ROS-Industrial universal_robot](https://github.com/ros-industrial/universal_robot/tree/39ad110d8f2e8f66856a201cca88aa7a7025e3eb/ur_description)，固定提交 `39ad110d8f2e8f66856a201cca88aa7a7025e3eb`，入口 `ur_description/urdf/ur5.xacro`，使用 `config/ur5/` 默认参数。本模型是 UR5（CB 系列），不是 UR5e。许可原文保存在 LICENSE。

上游建模贡献者及来源见同一提交的 `urdf/inc/ur_macro.xacro`：迁移到 YAML 配置的主要作者 Ludovic Delval；此前贡献者包括 Felix Messmer、Kelsey Hawkins、Wim Meeussen、Shaun Edwards、Nadia Hammoudeh Garcia、Dave Hershberger、G. vd. Hoorn 等；完整贡献者名单以上游文件为准。

本工程只将 package 路径改为相对路径，并将 visual 换成上游 collision 的 STL 和对应 origin，避免 DAE 渲染依赖。因此外观为简化网格；关节链、零位、限位、质量、惯量和原始碰撞几何均保留。manifest.json 保存 URDF 与所有网格的 SHA-256。无需安装 ROS 或运行时下载模型。

它是未按具体设备标定的通用仿真模型，不能作为实机校准数据；导入模型不代表已实现自碰撞规划或验证全部碰撞场景。逻辑 J1…J6 显式映射见 environment.lock.json；末端取 tool0。后续 DH FK 与 URDF 比较前须处理 base/base_link/tool0 等固定变换。

PyBullet适配补充：对base_link、base、flange、tool0四个无实体坐标链接显式写入零质量/零惯量，防止导入器自动赋予1kg虚假质量；实际机械连杆的上游惯量保持不变。
