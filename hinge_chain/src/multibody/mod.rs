//! # 多体动力学框架
//!
//! 基于广义坐标的多体动力学求解器，用于模拟由关节连接的刚体系统。
//!
//! ## 核心概念
//!
//! ### 广义坐标系统
//! - 使用最小坐标集 `q` 描述系统配置
//! - 每个铰链关节贡献一个自由度
//! - 避免了约束方程，提高计算效率
//!
//! ### 动力学管线
//! 1. **前向运动学**: q → (位置, 姿态)
//! 2. **速度计算**: 计算空间速度和运动子空间导数
//! 3. **力计算**: 重力、阻尼、科里奥利力
//! 4. **加速度求解**: M(q)·q̈ = τ
//! 5. **时间积分**: RK4 推进状态
//!
//! ## 核心算法
//!
//! - **CRBA** (Composite Rigid Body Algorithm): 计算质量矩阵 M(q)
//! - **RNE** (Recursive Newton-Euler): 计算广义力 τ
//! - **RK4** (Runge-Kutta 4阶): 高精度时间积分
//!
//! ## 快速开始
//!
//! ```rust,ignore
//! use multibody::*;
//!
//! // 1. 构建模型
//! let mut model = MultiBodyModel::new();
//! let body = RigidBody::new(mass, inertia);
//! let body_idx = model.add_body(body);
//!
//! let joint = HingeJoint {
//!     parent_body: -1,  // 固定在世界坐标系
//!     child_body: body_idx,
//!     axis: Vec3::X,    // 绕 X 轴旋转
//!     damping: 0.1,
//!     ..Default::default()
//! };
//! model.add_hinge_joint(joint);
//!
//! // 2. 初始化状态
//! let mut state = SimulationState::new(model.nq);
//! state.q[0] = 1.0;  // 设置初始角度
//!
//! // 3. 仿真循环
//! let dt = 0.001;  // 1ms 时间步长
//! loop {
//!     rk4_step(&mut model, &mut state, dt);
//! }
//! ```
//!
//! ## 参考
//!
//! 本实现参考了 MuJoCo 物理引擎的算法设计

// 模块声明
pub mod dynamics;     // 动力学算法 (RNE, CRBA)
pub mod geometry;     // 几何计算
pub mod integrator;   // 时间积分器
pub mod kinematics;   // 前向运动学
pub mod model;        // 数据结构定义
pub mod spatial_algebra;  // 6D 空间代数
pub mod subtree_com;  // 子树质心计算
pub mod velocity;     // 速度计算

// 导出常用类型，方便用户使用
pub use geometry::capsule_inertia;
pub use integrator::rk4_step;
pub use model::{HingeJoint, MultiBodyModel, RigidBody, SimulationState};

/// 重力常量 (m/s²)
///
/// 根据 MuJoCo XML 配置，重力沿 Z 轴负方向
pub const GRAVITY: bevy::math::Vec3 = bevy::math::Vec3::new(0.0, 0.0, -9.81);
