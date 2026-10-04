"""docking 命令行入口：参数化运行七自由度机械臂柔顺对接仿真实验。

仅使用标准库 argparse；实验编排逻辑唯一来源于
src/compliant_docking/orchestration/run_docking.py（experiments/run_docking.py
保留为兼容薄壳），惰性导入以保持启动时的环境变量设置顺序。

用法示例 / Examples:
    docking --quick                 # 2 秒快速冒烟（无渲染、无录帧）
    docking --scene scenes/iiwa14_docking.yaml --quick
    docking --duration 18 --render  # 完整时长 + 交互式渲染
    docking --controller hqp        # HQP-AC 约束自适应控制器（默认 impedance）
"""
import argparse
import os

from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene


def _load_run_docking():
    """加载主仿真编排模块（保持惰性导入，避免影响 MuJoCo/GL 环境变量）。"""
    from compliant_docking.orchestration import run_docking
    return run_docking


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="docking",
        description="七自由度机械臂柔顺对接仿真（MuJoCo × Pinocchio，多控制器可切换）",
    )
    parser.add_argument("--duration", type=float, default=None,
                        help="总时长；组合对接默认完整轨迹+保持，旧场景默认18秒")
    parser.add_argument("--dt", type=float, default=0.001,
                        help="仿真步长（秒），默认 0.001")
    parser.add_argument("--traj-duration", type=float, default=15.0,
                        help="对接轨迹时长（秒），默认 15.0")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE_PATH),
                        help="场景 YAML 路径（默认 iiwa14 对接场景）")
    parser.add_argument("--render", action=argparse.BooleanOptionalAction, default=False,
                        help="是否交互式渲染（默认 --no-render）")
    parser.add_argument("--record", action=argparse.BooleanOptionalAction, default=False,
                        help="是否离屏录帧并导出 MP4（默认 --no-record）")
    parser.add_argument("--quick", action="store_true",
                        help="快速冒烟测试：等价于 --duration 2.0")
    parser.add_argument("--controller", choices=["impedance", "se3_lie", "hqp"], default=None,
                        help="控制器：impedance=固定增益任务空间阻抗（默认）；"
                             "se3_lie=SE(3) Lie 群阻抗（Kim et al. 2025 T-RO，指数坐标+dexp 全链路）；"
                             "hqp=HQP-AC 约束自适应控制（Ren & Shan 2026 §3.2）")
    parser.add_argument("--out", help="HexFrame 运行结果目录；非空目录只能用于 --replay")
    parser.add_argument("--preview-only", action="store_true", help="HexFrame 完整路径与几何预检，不执行组装")
    parser.add_argument("--replay", action="store_true", help="从 --out 的已保存 HexFrame 状态回放")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    scene = load_scene(args.scene)
    from compliant_docking.assembly.config import AssemblyScene
    if isinstance(scene, AssemblyScene):
        if args.controller is not None or args.dt != .001 or args.duration is not None or args.render:
            raise ValueError("HexFrame uses its validated 1 ms, full 53 s assembly controller; use --record for saved-state video")
        from compliant_docking.assembly.runner import run
        return run(scene, output=args.out, record=args.record,
                   preview_only=args.preview_only or args.quick, replay=args.replay)
    if args.out or args.preview_only or args.replay:
        raise ValueError("--out/--preview-only/--replay currently apply to HexFrame assembly scenes")

    if args.quick:
        args.duration = 2.0
    elif args.duration is None and load_scene(args.scene).docking is None:
        args.duration = 18.0

    # 绘图仅落盘不弹窗：在导入 matplotlib 前锁定 Agg 后端 /
    # Figures are only saved to disk; pin the Agg backend before importing matplotlib
    os.environ.setdefault("MPLBACKEND", "Agg")

    run_docking = _load_run_docking()
    log = run_docking.main(
        render=args.render,
        record=args.record,
        dt=args.dt,
        traj_duration=args.traj_duration,
        duration=args.duration,
        scene_path=args.scene,
        controller=args.controller,
    )

    total_steps = len(log.t_list)
    final_err = log.error[-1] if log.error else float("nan")
    print(f"[docking] steps={total_steps} final_tracking_error={final_err:.6e} m")
    # 跟踪场景是柔顺对接的前置门禁：完整运行且门禁失败时以非零退出。
    # --quick 的短时运行会由实验层标为 INCOMPLETE，不视为失败。
    tracking_gate = getattr(log, "tracking_gate", None)
    if tracking_gate is not None and tracking_gate.status == "FAIL":
        return 2
    docking_gate = getattr(log, "docking_gate", None)
    if docking_gate is not None and docking_gate["status"] == "FAIL":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
