# Hinge Chain - 多体动力学仿真器

一个基于 Rust 实现的多体动力学仿真框架，用于模拟由铰链关节连接的刚体链。本项目参考了 MuJoCo 物理引擎的算法设计。

## 📋 项目概述

### ✨ 核心特性

- **广义坐标系统**：使用最小坐标集描述系统状态
- **前向运动学**：从关节角度计算刚体位置和姿态（四元数旋转）
- **前向动力学**：求解运动方程 M(q)·q̈ = τ
  - **CRBA** (Composite Rigid Body Algorithm): 计算质量矩阵
  - **RNE** (Recursive Newton-Euler): 计算广义力
- **空间代数**：基于 Lie 代数的 6D 运动和力表示
- **RK4 时间积分**：4阶精度的高精度时间演化
- **3D 可视化**：基于 Bevy 引擎的实时渲染
- **全面测试**：所有核心模块都有单元测试

### 🎯 演示案例

**胶囊链演示**：6 个胶囊体通过铰链关节串联，在重力作用下自由摆动。演示了：
- 多体系统的动力学仿真
- 科里奥利力和陀螺效应
- 长时间稳定仿真（RK4 积分）

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
│   │   ├── dynamics.rs              # 动力学算法 (CRBA, RNE)
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

### 1. 坐标系约定

本项目使用 **Bevy 标准坐标系**：
- **Y 轴向上** (+Y 是上方)
- 右手坐标系
- **重力方向**: (0, -9.81, 0) m/s²

### 2. 广义坐标 (Generalized Coordinates)

系统状态使用广义坐标表示：
- **q** (位置): 关节角度（弧度）
- **q̇** (速度): 关节角速度（弧度/秒）
- **q̈** (加速度): 关节角加速度（弧度/秒²）

每个铰链关节贡献 1 个自由度。

### 3. 空间代数 (Spatial Algebra)

根据Featherstone，使用 6D 向量描述刚体运动：
```
空间运动向量: [ω_x, ω_y, ω_z, v_x, v_y, v_z]
              角速度(3D)    线速度(3D)

空间力向量:   [τ_x, τ_y, τ_z, f_x, f_y, f_z]
              力矩(3D)      力(3D)
```

### 4. 动力学管线

完整的前向动力学流程：

```
1. 前向运动学 (forward_kinematics)
   输入: q (关节角度)
   输出: 每个刚体的位置和姿态

2. 子树质心计算 (compute_subtree_com)
   输出: 每个刚体子树的质心位置

3. 速度计算 (compute_velocities)
   输入: q, q̇
   输出: 空间速度、运动子空间、运动子空间导数

4. 力计算 (compute_generalized_forces)
   计算: 重力、阻尼、科里奥利力/离心力、陀螺力矩
   输出: τ (广义力)

5. 加速度求解 (compute_acceleration)
   方法: CRBA 计算质量矩阵 + 高斯消元求解
   求解: M(q)·q̈ = τ
   输出: q̈ (广义加速度)

6. 时间积分 (rk4_step)
   方法: Runge-Kutta 4阶（每步 4 次前向动力学）
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

- **方向键 ←/→**: 绕 Z 轴旋转相机
- **方向键 ↑/↓**: 缩放视野

### 数值稳定性

- **奇异值处理**: 避免除以接近零的数（阈值 1e-10）
- **RK4 积分**: 相比欧拉法有更好的稳定性
- **四元数归一化**: 每步归一化防止漂移
- **多子步积分**: 每帧 10 个子步提高稳定性

## 📚 参考文献

1. **Roy Featherstone** - *Rigid Body Dynamics Algorithms* (2008)
   - 空间代数基础
   - CRBA 和 RNE 算法

2. **MuJoCo Physics Engine**
   - 算法实现参考
   - 数据结构设计
