"""Render the metric table from tests/metrics.json (the single editable source)."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def render():
    data=json.loads((ROOT/'tests/metrics.json').read_text())
    text=['# 测试指标表','', '由 `python3 scripts/render_metrics.py` 生成。请编辑 tests/metrics.json 后重新生成。', '',
          '“已接入”表示存在可运行检查，不表示本轮或全部算法验收已通过；具体结果见运行目录summary.json。未定门槛须在所列节点冻结。','',
          '统一统计口径见 [统计规则 v1](statistics-rules.md)：有效样本误差汇总、全量分母成功率、三层耗时及超时记录。','',
          '| 编号 / 阶段 | 检查及入口 | 测试条件 | 阈值 / 单位 | 实现状态 |',
          '| --- | --- | --- | --- | --- |']
    for m in data['metrics']:
        state='已接入' if m['availability']=='implemented' else '待实现：'+m['pending_reason']
        cells=[m['id']+' / '+m['stage'],m['name']+' / '+str(m['test_id'] or '待接入'),m['conditions'],m['threshold']+' / '+m['unit'],state]
        text.append('| '+' | '.join(c.replace('|','\\|') for c in cells)+' |')
    text+=['','测量条件：seed=20260915、float64、BLAS单线程；记录提交、源码/模型/依赖锁哈希、版本、硬件、完整输入配置。统计包含失败与超时，不能只统计成功样本。','',
           'FK姿态误差采用相对旋转角；IK位置与姿态分别判定。关节轨迹必须检查连续段极值；路径采样误差检查不等于证明连续路径无碰撞。','',
           'Robotics Toolbox内置DH UR5与项目URDF可能存在base/tool固定坐标差异；BASE-02未通过前，参考输出只作依赖和数据准备。','',
           '来源：任务书第1～4阶段、docs/engineering-contract.md第9节及既有环境/ABI演示阈值。待定项和补充项在JSON中标明basis。','']
    return '\n'.join(text)
if __name__=='__main__':
    (ROOT/'docs/test-metrics.md').write_text(render())
