# 阻抗控制基础

阻抗控制使机械臂末端表现出"质量-阻尼-弹簧"的动力学特性，是柔顺对接的
理论基础。本文档给出关节空间与笛卡尔空间两种形式，并标注本仓库的对应实现。

## 关节空间阻抗控制

### 动力学模型

$$
M(q)\,\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau - \tau_{ext}
$$

### 期望动力学

$$
M_d\,\ddot{\tilde{q}} + D_d\,\dot{\tilde{q}} + K_d\,\tilde{q} = \tau_{ext},
\qquad \tilde{q} = q - q_d
$$

其中 $M_d, D_d, K_d$ 为期望的关节空间质量/阻尼/刚度矩阵。

### 控制律

跟随参考轨迹 $q_d(t), \dot q_d(t), \ddot q_d(t)$：

$$
\tau = M(q)M_d^{-1}\left(M_d\ddot{q}_d + D_d\dot{\tilde{q}} + K_d\tilde{q}\right)
+ \left(I - M(q)M_d^{-1}\right)\tau_{ext}
+ C(q,\dot{q})\dot{q} + g(q)
$$

**理想假设**：模型精确、关节力矩/角度/角速度可测、$\tau_{ext}$ 可测或可估计、理想力矩控制。

**工程简化**（本仓库历史路径采用）：

1. 去掉外力矩干扰项 $(I - M M_d^{-1})\tau_{ext}$；
2. 令 $\ddot q = \dot q = 0$ 并忽略 $M(q), M_d$。

## 笛卡尔空间阻抗控制

关节空间动力学改写为（外力经雅可比映射）：

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau - J^T(q)F_{ext}
$$

末端期望阻抗（平动三轴）：

$$
M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}
$$

本仓库的 CIC 控制器（`TaskSpaceController`）即此形式的实现，对应公式与
代码锚点见 [主仿真理论与数据流](simulation_flow.md)；带约束与自适应刚度的
HQP-AC 形式（论文 §3.2）见 [论文-代码对照](../paper_mapping.md)。

## 两类控制器的关系

| | CIC（`TaskSpaceController`） | HQP-AC（`HQPAdaptiveController`） |
|---|---|---|
| 刚度 | 常数（`ImpedanceConfig`，场景可覆盖） | 接触力 sigmoid 自适应（Eq.37–38） |
| 约束 | 软件力矩限幅 | 关节位置/速度/力矩 QP 硬约束 |
| 外力来源 | F/T 传感器 | 传感器或 PI 动量观测器 |
| 稳态预紧 | 摩擦 I 项自然建立 | 需显式 `preload_force` |
| 适用 | 基线对照 / 简单任务 | 力控精度与安全性要求高的对接 |
