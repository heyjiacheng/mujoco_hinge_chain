//! # Multibody Dynamics Framework
//!
//! 这是一个基于广义坐标的多体动力学求解框架，用于模拟由关节连接的刚体系统。
//!
//! ## 核心概念
//!
//! - **广义坐标 (Generalized Coordinates)**: 使用最小坐标集 q 描述系统状态
//! - **前向运动学 (Forward Kinematics)**: 从关节角度计算刚体位置和姿态
//! - **前向动力学 (Forward Dynamics)**: 从力/力矩计算加速度
//! - **时间积分 (Time Integration)**: 使用数值方法推进系统状态
//!
//! ## 算法参考
//!
//! 本框架实现的核心算法参考了以下经典算法：
//! - **RNE (Recursive Newton-Euler)**: 计算广义力
//! - **CRBA (Composite Rigid Body Algorithm)**: 计算质量矩阵
//! - **RK4 (Runge-Kutta 4th order)**: 时间积分
//!
//! ## 使用示例
//!
//! ```rust,ignore
//! use multibody::*;
//!
//! // 1. 创建模型
//! let mut model = MultiBodyModel::new();
//! let body = RigidBody::new(mass, inertia);
//! let joint = HingeJoint { ... };
//! model.add_body(body);
//! model.add_hinge_joint(joint);
//!
//! // 2. 初始化状态
//! let mut state = SimulationState::new(model.nq);
//!
//! // 3. 仿真循环
//! loop {
//!     rk4_step(&mut model, &mut state, dt);
//! }
//! ```

pub mod dynamics;
pub mod geometry;
pub mod integrator;
pub mod kinematics;
pub mod model;
pub mod spatial_algebra;
pub mod velocity;

// Re-export commonly used types
pub use dynamics::{compute_acceleration, compute_generalized_forces};
pub use geometry::capsule_inertia;
pub use integrator::rk4_step;
pub use kinematics::forward_kinematics;
pub use model::{HingeJoint, MultiBodyModel, RigidBody, SimulationState, SpatialInertia};
pub use velocity::compute_velocities;

/// Gravity constant (m/s^2)
/// 根据MuJoCo XML配置，重力沿Z轴负方向
pub const GRAVITY: bevy::math::Vec3 = bevy::math::Vec3::new(0.0, 0.0, -9.81);
