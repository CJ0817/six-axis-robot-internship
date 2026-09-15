# 统一统计规则补充验证

2026-09-15，云端 Linux。新增 `docs/statistics-rules.md`，指标 JSON 标记 `statistics_version=1`，指标表和测试说明链接同一规范。

补充了位置/姿态有效误差的计数、均值、RMSE、p95、最大值及空值约定；成功率固定分母、失败原因和三组独立统计；C、FFI、进程计时边界、分位数算法、预热、超时和实际等待时间。

执行：

```bash
python3 scripts/render_metrics.py
python3 -m unittest discover -s tests -p 'test_runner.py' -v
python3 scripts/run_tests.py --suite available --output results/test-runs/statistics-rules
```

结果：2/2 回归检查通过；C ABI、DIRECT 场景、RTB 依赖自检全部通过，进程超时数为 0。进程计时原始汇总保存在 `results/statistics-rules/summary.json`。其中子报告相对路径指向原始运行目录，再执行命令即可生成全部子报告。

超时回归使用模拟的 TimeoutExpired，验证计数、失败状态和实际等待字段，不代表进行了真实 120 秒超时或 C 工作进程强制终止测试。此次未实现 C/FFI 性能探针和未来求解器的分组统计器；规则中要求其接入时遵循统一字段，现阶段 C/FFI 耗时以 null/not_measured 报告。原先 RTB 准备自检数据不冒充完整算法成功率验收。
