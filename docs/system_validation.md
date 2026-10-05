# 完整对接/装配：系统验证与 Demo

HexFrame 使用模块 1 从存储抓取并安装到预锁定模块 2；六侧面接口并不表示六个模块。
正式流程为预检 → 存储/抓取交接 → 提起 → 转运 → 接触对接 → 持续就位 →
理想锁定 → 卸力释放 → 撤离。物理仿真 53 s；固定基座、零重力、刚性模型。

当前控制为 `assembly_admittance`：MuJoCo 偏置补偿关节伺服配合接触导纳。
Pinocchio 用于同源 IK、负载与刚体模型一致性检查。尚未集成单接口研究的 SE(3) 阻抗控制。
当前系统记录只证明这一配置的流程可用性。

## 三种入口语义

```bash
# 几何、双模型一致性与全路径可达性；结果必须标为 INCOMPLETE
uv run python -m experiments.system.hexframe precheck --out runs/hexframe_precheck_my_run

# 锁定原场景、1 ms 物理步长、53 s 时长与控制器；完整验收 + 独立审计
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.system.hexframe accept --out runs/hexframe_my_run

# 复用同一运行 model/rollout/phases，不重新模拟
MUJOCO_GL=egl uv run python -m experiments.system.hexframe replay --out runs/hexframe_my_run --record
```

验收核对六侧端口、CAD 质量惯量、关节限制、完整路径、状态顺序、真实导向/止挡载荷、
支撑与抓取交接、释放前卸力及撤离距离。非空目录拒绝新运行。失败退出 2；预检完成
退出 0 但 `validation.json` 为 `INCOMPLETE`。视频生成不升级验收状态。
正式 `docking --scene scenes/hexframe_assembly.yaml` 仍兼容并保持原限制。

回放先执行独立审计，再读取已保存状态和阶段。视频来自同一运行，不以当前默认路径替代记录。
系统证据包含 `validation.json`、`audit.json`、`runtime.json`、`source_manifest.json`、
模型、状态与接触 trace。详细门禁与物理边界见[HexFrame 正式场景](hexframe_assembly.md)。

## 独立扩展验证

| 编号 | 唯一工作包 | 归因与限制 |
|---|---|---|
| P0 | 物理步长 1 → 0.5 ms | 保持阶段时长、增益与 10 ms 接触 IK 更新；检查事件与验收量漂移 |
| P1 | pick/seed 布局扰动 | 每点重新规划与全流程审计；离散范围 |
| P2 | 测量噪声与 raw_strict/filtered_debounce 判定策略 | 噪声与判定策略独立标识；不可替代控制主线证据 |

P0 中关节伺服、轴向导纳积分、滤波与测量判定仍每物理步运行，半步长同时改变这些离散更新频率。
因此它是系统步长复核，不等同于 Petal 固定 0.5 ms 控制周期及反馈延迟的纯物理步长协议。

入口位于 `experiments/system/validation/`。
[历史结果与复现](historical_evidence.md)保留；正式验收仍用 raw_strict，不自动晋升候选门禁。

## 待集成能力

后续接入研究策略时明确末端控制点、负载切换、反馈来源、控制周期与抓取约束，
先完成局部行为回归，再全流程验收。原导纳流程保留为系统基线，新结果独立归档。
真实锁销/预紧、自由漂浮基座、重力与实机推广分别验证。
