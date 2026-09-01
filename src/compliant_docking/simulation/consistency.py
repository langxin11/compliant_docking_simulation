"""consistency.py - MuJoCo × Pinocchio 一致性控制器（关节空间 PD + 逆动力学）。"""
import os
from typing import Optional

import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin

from ..models import ASSETS_DIR, PIN_URDF, load_pin_model


class RobotController:
    def __init__(self, model_path: str, urdf_path: str):
        """
        初始化机器人控制器（MuJoCo + Pinocchio）：用于物理仿真与动力学计算
        Args:
            model_path: MuJoCo模型文件路径
            urdf_path: URDF文件路径
        """
        # 检查文件是否存在
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"MuJoCo model file not found: {model_path}")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        # 加载MuJoCo模型（用于物理仿真）
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            raise RuntimeError(f"Failed to load MuJoCo model: {str(e)}")

        # 创建Pinocchio模型（用于动力学/雅可比计算）；
        # 重力置零由 load_pin_model 统一处理（控制侧去重力，仿真环境可能仍生效）
        try:
            self.pin_model = load_pin_model(urdf_path)
            self.pin_data = self.pin_model.createData()
        except Exception as e:
            raise RuntimeError(f"Failed to load Pinocchio model: {str(e)}")
        
        # 控制参数
        self.Kp = 100.0  # P增益
        self.Kd = 20.0   # D增益
        
        # 机器人参数
        self.nq = self.pin_model.nq  # 关节数量
        self.joint_limits = {
            'lower': self.pin_model.lowerPositionLimit,
            'upper': self.pin_model.upperPositionLimit,
            'velocity': self.pin_model.velocityLimit,
            'torque': self.pin_model.effortLimit
        }
        
        # 记录数据用于绘图（关节角/速度/力矩/误差）
        self.reset_logs()

    def reset_logs(self):
        """重置记录数据"""
        self.time_log = []
        self.q_desired_log = []
        self.q_actual_log = []
        self.dq_actual_log = []
        self.tau_log = []
        self.error_log = []
        
    def forward_dynamics(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        计算前向动力学（ABA）：返回关节加速度
        """
        pin.computeAllTerms(self.pin_model, self.pin_data, q, dq)
        ddq = pin.aba(self.pin_model, self.pin_data, q, dq, tau)
        return ddq
        
    def inverse_dynamics(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
        """
        计算逆动力学（RNEA）：返回关节力矩
        """
        tau = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)
        return tau
        
    def clamp_torque(self, tau: np.ndarray) -> np.ndarray:
        """限制力矩在允许范围内"""
        return np.clip(tau, -self.joint_limits['torque'], self.joint_limits['torque'])
        
    def compute_control(self, q_desired: np.ndarray, dq_desired: np.ndarray, 
                       ddq_desired: np.ndarray, q_current: np.ndarray, 
                       dq_current: np.ndarray) -> np.ndarray:
        """计算控制输出（关节空间 PD + 逆动力学前馈）"""
        # 检查输入维度
        if any(arr.shape != (self.nq,) for arr in [q_desired, dq_desired, ddq_desired, q_current, dq_current]):
            raise ValueError("Input arrays must match the number of joints")
            
        # 计算误差
        q_error = q_desired - q_current
        dq_error = dq_desired - dq_current
        
        # 计算期望加速度（PD控制）
        ddq = ddq_desired + self.Kp * q_error + self.Kd * dq_error
        
        # 计算所需关节力矩（推荐：使用当前状态与期望加速度）
        # tau = self.inverse_dynamics(q_current, dq_current, ddq)
        # 当前实现使用期望状态/加速度，偏差大时可能不一致；可按上行切换
        tau = self.inverse_dynamics(q_desired, dq_desired, ddq_desired)
        return tau

    def generate_quintic_trajectory(self, t: float, t0: float, tf: float, 
                                  q0: np.ndarray, qf: np.ndarray,
                                  v0: np.ndarray = None, vf: np.ndarray = None,
                                  a0: np.ndarray = None, af: np.ndarray = None) -> tuple:
        """
        生成五次多项式轨迹（满足端点速度/加速度为零）
        Args:
            t: 当前时间点
            t0: 起始时间
            tf: 终止时间
            q0: 起始位置
            qf: 终止位置
            v0: 起始速度，默认为0
            vf: 终止速度，默认为0
            a0: 起始加速度，默认为0
            af: 终止加速度，默认为0
        Returns:
            tuple: (位置，速度，加速度)
        """
        # 设置默认值
        if v0 is None:
            v0 = np.zeros_like(q0)
        if vf is None:
            vf = np.zeros_like(qf)
        if a0 is None:
            a0 = np.zeros_like(q0)
        if af is None:
            af = np.zeros_like(qf)

        # 归一化时间
        T = tf - t0
        if T <= 0:
            raise ValueError("Final time must be greater than initial time")
        
        # 计算归一化当前时间
        s = (t - t0) / T
        if s < 0:
            return q0, v0, a0
        elif s > 1:
            return qf, vf, af

        # 五次多项式系数计算
        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, 1, 1, 1, 1, 1],
            [0, 1, 2, 3, 4, 5],
            [0, 0, 2, 6, 12, 20]
        ])

        # 计算每个关节的轨迹
        q = np.zeros_like(q0)
        dq = np.zeros_like(q0)
        ddq = np.zeros_like(q0)

        for i in range(len(q0)):
            # 边界条件向量
            b = np.array([
                q0[i],
                v0[i] * T,
                a0[i] * T * T,
                qf[i],
                vf[i] * T,
                af[i] * T * T
            ])

            # 求解系数
            x = np.linalg.solve(A, b)

            # 计算位置
            s_vec = np.array([1, s, s**2, s**3, s**4, s**5])
            q[i] = np.dot(s_vec, x)

            # 计算速度
            ds_vec = np.array([0, 1, 2*s, 3*s**2, 4*s**3, 5*s**4]) / T
            dq[i] = np.dot(ds_vec, x)

            # 计算加速度
            dds_vec = np.array([0, 0, 2, 6*s, 12*s**2, 20*s**3]) / (T * T)
            ddq[i] = np.dot(dds_vec, x)

        return q, dq, ddq

    def generate_trajectory(self, t: float, trajectory_type: str = 'sine') -> tuple:
        """
        生成不同类型的轨迹
        Args:
            t: 时间点
            trajectory_type: 轨迹类型 ('sine', 'circle', 'square', 'quintic')
        Returns:
            tuple: (位置，速度，加速度)
        """
        if trajectory_type == 'quintic':
            # 定义五次多项式轨迹的起始和终止状态
            t0 = 0.0
            tf = 5.0  # 轨迹总时间
            q0 = np.zeros(self.nq)  # 起始位置
            qf = np.array([0.7, 0.6, 0.5, 0.4, 0.3, 0.2,0.1])  # 终止位置
            v0 = np.zeros(self.nq)  # 起始速度
            vf = np.zeros(self.nq)  # 终止速度
            a0 = np.zeros(self.nq)  # 起始加速度
            af = np.zeros(self.nq)  # 终止加速度

            return self.generate_quintic_trajectory(t, t0, tf, q0, qf, v0, vf, a0, af)
            
        elif trajectory_type == 'sine':
            omega = 2 * np.pi  # 角频率
            q = np.array([
                0.5 * np.sin(omega * t),
                0.3 * np.sin(omega * t + np.pi/4),
                0.4 * np.sin(omega * t + np.pi/3),
                0.3 * np.sin(omega * t + np.pi/2),
                0.2 * np.sin(omega * t + 2*np.pi/3),
                0.3 * np.sin(omega * t + 3*np.pi/4),
                0.4 * np.sin(omega * t + np.pi)
            ])
            # 计算速度（一阶导数）：dq/dt = amplitude * omega * cos(omega*t + phase)
            dq = np.array([
                0.5 * omega * np.cos(omega * t),
                0.3 * omega * np.cos(omega * t + np.pi/4),
                0.4 * omega * np.cos(omega * t + np.pi/3),
                0.3 * omega * np.cos(omega * t + np.pi/2),
                0.2 * omega * np.cos(omega * t + 2*np.pi/3),
                0.3 * omega * np.cos(omega * t + 3*np.pi/4),
                0.4 * omega * np.cos(omega * t + np.pi)
            ])
            # 计算加速度（二阶导数）：d²q/dt² = -amplitude * omega² * sin(omega*t + phase)
            ddq = np.array([
                -0.5 * omega**2 * np.sin(omega * t),
                -0.3 * omega**2 * np.sin(omega * t + np.pi/4),
                -0.4 * omega**2 * np.sin(omega * t + np.pi/3),
                -0.3 * omega**2 * np.sin(omega * t + np.pi/2),
                -0.2 * omega**2 * np.sin(omega * t + 2*np.pi/3),
                -0.3 * omega**2 * np.sin(omega * t + 3*np.pi/4),
                -0.4 * omega**2 * np.sin(omega * t + np.pi)
            ])
            return q, dq, ddq
        
        elif trajectory_type == 'circle':
            omega = 2 * np.pi  # 角频率
            q = np.array([
                0.3 * np.cos(omega * t),
                0.3 * np.sin(omega * t),
                0.2 * np.cos(omega * t),
                0.2 * np.sin(omega * t),
                0.1 * np.cos(omega * t),
                0.1 * np.sin(omega * t),
                0.1 * np.cos(omega * t)
            ])
            # 计算速度：cos的导数是-sin，sin的导数是cos
            dq = np.array([
                -0.3 * omega * np.sin(omega * t),
                0.3 * omega * np.cos(omega * t),
                -0.2 * omega * np.sin(omega * t),
                0.2 * omega * np.cos(omega * t),
                -0.1 * omega * np.sin(omega * t),
                0.1 * omega * np.cos(omega * t),
                -0.1 * omega * np.sin(omega * t)
            ])
            # 计算加速度：再次求导
            ddq = np.array([
                -0.3 * omega**2 * np.cos(omega * t),
                -0.3 * omega**2 * np.sin(omega * t),
                -0.2 * omega**2 * np.cos(omega * t),
                -0.2 * omega**2 * np.sin(omega * t),
                -0.1 * omega**2 * np.cos(omega * t),
                -0.1 * omega**2 * np.sin(omega * t),
                -0.1 * omega**2 * np.cos(omega * t)
            ])
            return q, dq, ddq
            
    def run_simulation(self, trajectory_type: str = 'quintic', 
                      duration: float = 5.0, dt: float = 0.001,
                      render: bool = False):
        """运行仿真"""
        steps = int(duration / dt)
        self.reset_logs()

        self.data.qpos[:7] = np.zeros(self.nq)
        self.data.qvel[:7] = np.zeros(self.nq)
        
        
        viewer = None
        if render:
            viewer = mujoco.viewer.launch_passive(self.model, self.data)

        for i in range(steps):
            t = i * dt
            
            # 获取期望轨迹
            q_d, dq_d, ddq_d = self.generate_trajectory(t, trajectory_type)
            
            # 获取当前状态
            q = self.data.qpos[:7]
            dq = self.data.qvel[:7]
            
            # 计算控制输出
            tau = self.compute_control(q_d, dq_d, ddq_d, q, dq)
            
            # 应用控制
            self.data.ctrl[:7] = tau
            
            # 记录数据
            self.time_log.append(t)
            self.q_desired_log.append(q_d.copy())
            self.q_actual_log.append(q.copy())
            self.dq_actual_log.append(dq.copy())
            self.tau_log.append(tau.copy())
            self.error_log.append(np.linalg.norm(q_d - q))
            
            # 推进仿真
            mujoco.mj_step(self.model, self.data)
            
            # 渲染（如果启用）
            if render and i % 10 == 0:  # 每10步渲染一次
                viewer.sync()

    def plot_results(self, save_path: Optional[str] = None):
        """
        绘制结果对比图
        Args:
            save_path: 如果提供，将图像保存到指定路径
        """
        time_array = np.array(self.time_log)
        q_desired_array = np.array(self.q_desired_log)
        q_actual_array = np.array(self.q_actual_log)
        tau_array = np.array(self.tau_log)
        error_array = np.array(self.error_log)
        
        # 创建子图
        set_plot_config()
        fig = plt.figure(figsize=(15, 10))
        gs = plt.GridSpec(3, 2)
        
        # 角度轨迹图
        ax1 = fig.add_subplot(gs[0:2, 0])
        for i in range(self.nq):
            ax1.plot(time_array, q_desired_array[:, i], '--', label=f'Desired j{i+1}')
            ax1.plot(time_array, q_actual_array[:, i], '-', label=f'Actual j{i+1}')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Joint Angles (rad)')
        ax1.grid(True)
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # 跟踪误差图
        ax2 = fig.add_subplot(gs[2, 0])
        ax2.plot(time_array, error_array, 'r-', label='Tracking Error')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error Norm')
        ax2.grid(True)
        ax2.legend()
        # 控制力矩图
        ax3 = fig.add_subplot(gs[:, 1])
        for i in range(self.nq):
            ax3.plot(time_array, tau_array[:, i], '-', label=f'Joint {i+1}')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Control Torque (Nm)')
        ax3.grid(True)
        ax3.legend()
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path)
        plt.show()

def set_plot_config():
    """Set the configuration for matplotlib plots to ensure consistent styling."""
    config = {
    # 不启用真实 LaTeX，而是使用 mathtext 模拟 (无需系统安装 LaTeX)
    "text.usetex": False,

    # =============================
    # 字体设置（非常重要）
    # =============================

    # 可以选择 "serif" 或 "sans-serif"（默认类别），系统会在下面列表中匹配字体。
    "font.family": "serif",  # ✅ 建议设为 serif（衬线体，更接近 LaTeX 风格）

    # serif 字体列表（衬线体，适合论文/公式类）
    "font.serif": [
        "SimSun",               # 宋体（Windows 中文字体）
        "NSimSun",              # 新宋体（Windows 中文字体）
        "Noto Serif CJK SC",    # Noto Serif 中文（Linux / macOS 常用）
        "Songti SC",            # macOS 常见宋体
        "Times New Roman",      # 英文字体（LaTeX 风格）
        "DejaVu Serif"          # fallback 英文字体
    ],

    # sans-serif 字体列表（无衬线体，备用用于界面/标签）
    "font.sans-serif": [
        "SimHei",               # 黑体（Windows）
        "Microsoft YaHei",      # 微软雅黑（Windows）
        "Noto Sans CJK SC",     # Noto Sans 中文（Linux / macOS）
        "Heiti TC",             # 黑体（macOS）
        "Arial Unicode MS",     # 旧版 macOS 中文字体
        "Arial",                # 英文字体（常见）
        "DejaVu Sans"           # fallback 英文字体
    ],

    # =============================
    # 字号控制
    # =============================
    "font.size": 10,
    "axes.labelsize": 10,     # 坐标轴标签字号
    "legend.fontsize": 8,    # 图例字号
    "xtick.labelsize": 8,    # x 轴刻度字号
    "ytick.labelsize": 8,    # y 轴刻度字号

    # =============================
    # 数学字体设置
    # =============================

    # STIX 数学字体与 Times Roman 风格接近
    "mathtext.fontset": "stix",
    "mathtext.rm": "Times New Roman",
    "mathtext.it": "Times New Roman:italic",
    "mathtext.bf": "Times New Roman:bold",

    # =============================
    # 图像质量与布局
    # =============================
    "figure.dpi": 150,        # 屏幕显示分辨率（适中）
    "savefig.dpi": 300,       # 保存图片的分辨率（论文质量）
    "figure.autolayout": True,# 自动调整布局，防止标签被裁剪

    # =============================
    # 显示修正
    # =============================
    "axes.unicode_minus": False  # 解决负号 '-' 显示成方块的问题
    }

    # 应用配置
    plt.rcParams.update(config)


def main():
    # 创建控制器实例
    controller = RobotController(
        model_path=str(ASSETS_DIR / "iiwa14.xml"),
        urdf_path=str(PIN_URDF)
    )
    
    # 运行不同轨迹的仿真
    trajectories = ['sine']
    for traj in trajectories:
        print(f"\nRunning simulation with {traj} trajectory...")
        controller.run_simulation(
            trajectory_type=traj,
            duration=5.0,
            render=True
        )
        controller.plot_results(save_path=f"results_{traj}.png")

if __name__ == "__main__":
    main()
