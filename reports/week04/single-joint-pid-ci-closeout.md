# PID提交与CI收尾（2026-10-15）

代码、配置、接口说明、CMake/统一测试入口和本地原始证据由 `246737cafedfb84dc64f4a6bf3729fd2c54f7027` 提交并推送至main；原先 `61eb06b` 的成功记录不作为PID验收依据。

首轮PID CI的C编译与积分基线通过，PyBullet导入因缺少NumPy失败，保留 [失败记录](../../results/single-joint-pid-ci/first-run-failure.json)。提交 `8d32bfa87738793128c7ba4c939f7deee0b6af7e` 补装与项目requirements.lock一致的numpy==1.26.4，算法、PID参数和通过阈值均未改变。

## 新代码的云端实测

[Single joint PID运行38714341882](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38714341882) 对应8d32bfa，严格C11编译、230项接口/算法回归、三类积分模型试验、三类UR5 J1力矩试验和2项测试入口回归全部成功。每种被控对象保存2400条采样记录，合计4800条；每条包含参考、反馈、控制输出、积分与限幅状态。

同一提交的 [Environment verification运行38714342025](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38714342025) 已完成且conclusion=success，environment与robotics-toolbox-baseline两个job均成功；没有引用旧主分支的检查结果。

| UR5 J1力矩试验 | RMSE（rad） | 末值误差绝对值（rad） | 最后1秒平均偏差（rad） | 超调（%） | 调节时间（s） |
| --- | ---: | ---: | ---: | ---: | ---: |
| 阶跃 | 0.024189489 | 0.001199625 | -0.001321824 | 5.66471 | 1.22 |
| 正弦 | 0.001656265 | 0.000001792 | 0.000001973 | 不适用 | 0.37 |
| 定位 | 0.005427988 | 0.000259481 | -0.000285913 | 5.72682 | 0.83 |

调节时间是进入规定误差带并保持到记录结束的时间；正弦从5s停止正弦后计算。稳态偏差仅表示最后1秒均值，不宣称数学稳态。功能阈值仍为运行前冻结的末值≤0.02rad等条件，不代表CTRL-01的20条轨迹或完整CTRL-02验收。

原始证据来自 [artifact 11962185845](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38714341882/artifacts/11962185845)，ZIP SHA256=`8c8a9cd4b1133aae3dc35029102aeb50d9fa85752eb293109932152f92e2b404`。本地下载后校验归档哈希，并按report.json核对全部6个CSV哈希；只将本次运行重新生成的报告/日志/CSV/SVG归档到 `results/single-joint-pid-ci/8d32bfa/`。artifact中继承自仓库的Windows validation记录不作为云端记录归档。

## 证明范围

本轮交付为独立单关节纯PID功能基线：PyBullet J1使用自研PID力矩输出，其余关节保持、重力关闭；保持原有参数、限幅与复位约定。六轴前馈、扰动恢复、控制故障状态机、正式20条轨迹及墙钟实时周期仍未验收。新增证据不改写历史运动学或本机GUI验收。

最后归档提交的CI状态应以该提交自己的GitHub Actions为准，不能仅引用父提交的成功记录。提交与运行映射保存在 `results/single-joint-pid-ci/ci-provenance.json`；后续新提交的完整通过记录可直接从GitHub主分支检查页核对。
