# Hinge Chain - 多体动力学仿真器

一个基于 Rust 实现的多体动力学仿真框架，用于模拟由铰链关节连接的刚体链。本项目参考了 MuJoCo 物理引擎的算法设计。

## 📋 项目概述

本项目实现了一个完整的多体动力学求解器，包括：

- **广义坐标系统**：使用最小坐标集描述系统状态
- **前向运动学**：从关节角度计算刚体位置和姿态
- **前向动力学**：求解运动方程 M(q)·q̈ = τ
- **RK4 时间积分**：高精度的时间演化
- **3D 可视化**：基于 Bevy 引擎的实时渲染

### 演示案例

项目包含一个胶囊链演示：6 个胶囊通过铰链关节连接，在重力作用下摆动。

## 🏗️ 代码结构

```
hinge_chain/
├── src/
│   ├── main.rs                      # 可视化和主循环
│   ├── multibody/                   # 核心物理引擎
│   │   ├── mod.rs                   # 模块导出和文档
│   │   ├── model.rs                 # 数据结构定义
│   │   ├── spatial_algebra.rs       # 6D 空间代数
│   │   ├── kinematics.rs            # 前向运动学
│   │   ├── velocity.rs              # 速度计算
│   │   ├── dynamics.rs              # 动力学算法 (RNE, CRBA)
│   │   ├── integrator.rs            # 时间积分器
│   │   ├── geometry.rs              # 几何工具
│   │   └── subtree_com.rs           # 子树质心计算
│   └── demos/
│       ├── mod.rs
│       └── hinge_chain.rs           # 胶囊链演示
├── Cargo.toml
└── README.md
```

## 🔬 核心概念

### 1. 广义坐标 (Generalized Coordinates)

系统状态使用广义坐标表示：
- **q** (位置): 关节角度（弧度）
- **q̇** (速度): 关节角速度（弧度/秒）
- **q̈** (加速度): 关节角加速度（弧度/秒²）

每个铰链关节贡献 1 个自由度。

### 2. 空间代数 (Spatial Algebra)

使用 6D 向量描述刚体运动：
```
空间运动向量: [ω_x, ω_y, ω_z, v_x, v_y, v_z]
              角速度(3D)    线速度(3D)

空间力向量:   [τ_x, τ_y, τ_z, f_x, f_y, f_z]
              力矩(3D)      力(3D)
```

### 3. 动力学管线

完整的前向动力学流程：

```
1. 前向运动学 (forward_kinematics)
   输入: q (关节角度)
   输出: 每个刚体的位置和姿态

2. 速度计算 (compute_velocities)
   输入: q, q̇
   输出: 空间速度、运动子空间导数

3. 力计算 (compute_generalized_forces)
   计算: 重力、阻尼、科里奥利力/离心力
   输出: τ (广义力)

4. 加速度求解 (compute_acceleration)
   求解: M(q)·q̈ = τ
   输出: q̈ (广义加速度)

5. 时间积分 (rk4_step)
   方法: Runge-Kutta 4阶
   更新: q, q̇
```

## 🧮 核心算法

### CRBA (Composite Rigid Body Algorithm)

计算质量矩阵 M(q)：

1. **初始化**: 每个刚体的空间惯性
2. **后向递推**: 累积子树惯性（叶→根）
3. **前向递推**: 计算 M[i,j] 元素（根→叶）

质量矩阵性质：
- 对称正定
- 配置相关 M = M(q)
- 大小为 nq × nq

### RNE (Recursive Newton-Euler)

计算广义力 τ：

**重力力矩**：
```
τ_i = Σ (r_j × F_gravity) · axis_i
      j∈subtree(i)
```
关节 i 必须支撑所有下游刚体的重量。

**科里奥利/离心力**：
```
τ_gyro = -ω × (I·ω)
```
陀螺效应，由旋转引起。

**阻尼力**：
```
τ_damp = -damping · q̇
```
模拟摩擦和能量耗散。

### RK4 时间积分

4 阶 Runge-Kutta 方法：

```
k₁ = f(t, y)
k₂ = f(t + h/2, y + h/2·k₁)
k₃ = f(t + h/2, y + h/2·k₂)
k₄ = f(t + h, y + h·k₃)
y_{n+1} = y_n + h/6·(k₁ + 2k₂ + 2k₃ + k₄)
```

- 局部误差: O(h⁵)
- 全局误差: O(h⁴)
- 每步需要 4 次动力学计算

## 🚀 使用方法

### 编译和运行

```bash
cd hinge_chain
cargo run --release
```

### 交互控制

- **鼠标左键拖拽**: 旋转视角
- **鼠标滚轮**: 缩放
- **ESC**: 退出

### 代码示例

```rust
use hinge_chain::multibody::*;

// 1. 创建刚体
let mass = 1.0;
let inertia = capsule_inertia(length, radius, mass);
let body = RigidBody::new(mass, inertia);

// 2. 构建模型
let mut model = MultiBodyModel::new();
let body_idx = model.add_body(body);

// 3. 添加关节
let joint = HingeJoint {
    parent_body: -1,        // 固定在世界坐标系
    child_body: body_idx,
    axis: Vec3::X,          // 绕 X 轴旋转
    body_offset: Vec3::new(0.0, 0.0, length/2.0),
    joint_offset: Vec3::new(0.0, 0.0, -length/2.0),
    damping: 0.5,
    armature: 0.0,
    ..Default::default()
};
model.add_hinge_joint(joint);

// 4. 初始化状态
let mut state = SimulationState::new(model.nq);

// 5. 仿真循环
let dt = 0.001;  // 1ms
loop {
    rk4_step(&mut model, &mut state, dt);
}
```

## 📊 性能特征

| 特性 | 值 |
|------|-----|
| 时间步长 | 1ms (可调) |
| 积分器 | RK4 (4阶精度) |
| 质量矩阵 | CRBA O(n²) |
| 线性求解器 | 高斯消元 O(n³) |
| 适用规模 | 小型系统 (< 100 DOF) |

## 🔧 技术细节

### 坐标系约定

- **世界坐标系**: 右手系，Z 轴向上
- **重力方向**: -Z 方向 (0, 0, -9.81 m/s²)
- **关节轴**: 在父体坐标系中定义
- **空间向量**: [角量, 线量] 顺序

### 数值稳定性

- 质量矩阵求解使用部分主元高斯消元
- 避免除以接近零的数（使用阈值 1e-10）
- RK4 相比欧拉法有更好的能量守恒

### 限制和假设

1. **树状拓扑**: 不支持闭环机构
2. **刚体假设**: 物体不变形
3. **无碰撞**: 未实现碰撞检测和接触力
4. **单关节类型**: 仅支持铰链关节
5. **质心假设**: 胶囊质心在几何中心

## 📚 参考文献

1. **Roy Featherstone** - *Rigid Body Dynamics Algorithms* (2008)
   - 空间代数基础
   - CRBA 和 RNE 算法

2. **MuJoCo Physics Engine**
   - 算法实现参考
   - 数据结构设计

3. **NumPy-ML Implementation**
   - Python 原型实现

## 🛠️ 未来改进

- [ ] 实现 LDL^T 分解优化线性求解
- [ ] 添加更多关节类型（球关节、滑动关节）
- [ ] 实现碰撞检测和接触力
- [ ] 支持外力输入
- [ ] 添加更多测试用例
- [ ] 性能优化（缓存质量矩阵）
- [ ] 支持闭环机构

## 📝 许可证

本项目仅供学习和研究使用。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

**作者**: Jiadeng Xu
**日期**: 2025
