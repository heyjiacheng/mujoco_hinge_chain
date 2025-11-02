//! # 时间积分模块
//!
//! 实现数值积分算法，用于推进多体系统的时间演化

use super::dynamics::{compute_acceleration, compute_generalized_forces};
use super::kinematics::forward_kinematics;
use super::model::{MultiBodyModel, SimulationState};
use super::velocity::compute_velocities;

/// 前向动力学：计算当前状态下的加速度
///
/// 这是完整的前向动力学管线，对应MuJoCo的`mj_forward()`函数。
///
/// ## 算法流程
///
/// 1. **Forward Kinematics**: q → (位置, 姿态)
/// 2. **Compute Forces**: 计算广义力 τ (重力, 阻尼, etc.)
/// 3. **Compute Acceleration**: 求解 q̈ = M^(-1) * τ
///
/// ## 参数
/// - `model`: 多体模型
/// - `state`: 仿真状态（输入q和q̇，输出q̈）
///
fn forward_dynamics(model: &mut MultiBodyModel, state: &mut SimulationState) {
    // 步骤1: 前向运动学 - 计算位置和姿态
    // 对应 MuJoCo 的 mj_kinematics()
    forward_kinematics(model, state);

    // 步骤2: ⭐关键修复⭐ 计算空间速度和cdof_dot
    // 对应 MuJoCo 的 mj_comVel()
    // 这一步计算科里奥利/离心力所需的 cdof_dot
    compute_velocities(model, state);

    // 步骤3: 计算广义力 (重力 + 科里奥利/离心 + 阻尼)
    // 对应 MuJoCo 的 mj_passive() + mj_rne()
    let mut qfrc = vec![0.0; model.nq];
    compute_generalized_forces(model, state, &mut qfrc);
    state.qfrc = qfrc;

    // 步骤4: 计算广义加速度 q̈ = M^(-1) * τ
    // 对应 MuJoCo 的 mj_fwdAcceleration()
    compute_acceleration(model, state);
}

/// RK4 (Runge-Kutta 4阶) 时间积分器
///
/// ## 理论背景
///
/// RK4是一种高精度的显式数值积分方法，对于常微分方程：
/// ```text
/// dy/dt = f(t, y)
/// ```
///
/// RK4使用加权平均的斜率来更新状态：
/// ```text
/// k₁ = f(t, y)
/// k₂ = f(t + h/2, y + h/2*k₁)
/// k₃ = f(t + h/2, y + h/2*k₂)
/// k₄ = f(t + h, y + h*k₃)
/// y_{n+1} = y_n + h/6*(k₁ + 2k₂ + 2k₃ + k₄)
/// ```
///
/// ## 应用到多体系统
///
/// 对于二阶系统（位置和速度）：
/// ```text
/// y = [q, q̇]
/// dy/dt = [q̇, q̈] = f(t, y)
/// ```
///
/// 我们需要计算4个中间状态的导数（k1, k2, k3, k4），每次都调用
/// 前向动力学来获得当前状态的加速度。
///
/// ## 精度
///
/// RK4的局部截断误差为 O(h⁵)，全局误差为 O(h⁴)，比欧拉法的
/// O(h²) 精度高很多，适合长时间仿真。
///
/// ## 与MuJoCo的对应关系
///
/// 本实现完全对应MuJoCo的`mj_RungeKutta()`函数（在src/engine/engine_forward.c:904）。
///
/// ## 参数
/// - `model`: 多体模型
/// - `state`: 仿真状态（会被更新）
/// - `dt`: 时间步长（秒）
///
pub fn rk4_step(model: &mut MultiBodyModel, state: &mut SimulationState, dt: f32) {
    let nq = model.nq;

    // 保存初始状态
    let q0 = state.q.clone();
    let qvel0 = state.qvel.clone();

    // k值的临时存储
    // k_vel 对应 q̇, k_acc 对应 q̈
    let mut k1_vel = vec![0.0; nq];
    let mut k1_acc = vec![0.0; nq];
    let mut k2_vel = vec![0.0; nq];
    let mut k2_acc = vec![0.0; nq];
    let mut k3_vel = vec![0.0; nq];
    let mut k3_acc = vec![0.0; nq];
    let mut k4_vel = vec![0.0; nq];
    let mut k4_acc = vec![0.0; nq];

    // ==================== Stage 1: k1 = f(t, y) ====================
    forward_dynamics(model, state);
    k1_vel.copy_from_slice(&state.qvel);
    k1_acc.copy_from_slice(&state.qacc);

    // =============== Stage 2: k2 = f(t + h/2, y + h/2*k1) ===============
    for i in 0..nq {
        state.q[i] = q0[i] + 0.5 * dt * k1_vel[i];
        state.qvel[i] = qvel0[i] + 0.5 * dt * k1_acc[i];
    }
    forward_dynamics(model, state);
    k2_vel.copy_from_slice(&state.qvel);
    k2_acc.copy_from_slice(&state.qacc);

    // =============== Stage 3: k3 = f(t + h/2, y + h/2*k2) ===============
    for i in 0..nq {
        state.q[i] = q0[i] + 0.5 * dt * k2_vel[i];
        state.qvel[i] = qvel0[i] + 0.5 * dt * k2_acc[i];
    }
    forward_dynamics(model, state);
    k3_vel.copy_from_slice(&state.qvel);
    k3_acc.copy_from_slice(&state.qacc);

    // ================ Stage 4: k4 = f(t + h, y + h*k3) ================
    for i in 0..nq {
        state.q[i] = q0[i] + dt * k3_vel[i];
        state.qvel[i] = qvel0[i] + dt * k3_acc[i];
    }
    forward_dynamics(model, state);
    k4_vel.copy_from_slice(&state.qvel);
    k4_acc.copy_from_slice(&state.qacc);

    // ========== Final update: y_next = y + h/6*(k1 + 2*k2 + 2*k3 + k4) ==========
    for i in 0..nq {
        state.q[i] = q0[i] + dt / 6.0 * (k1_vel[i] + 2.0 * k2_vel[i] + 2.0 * k3_vel[i] + k4_vel[i]);
        state.qvel[i] = qvel0[i] + dt / 6.0 * (k1_acc[i] + 2.0 * k2_acc[i] + 2.0 * k3_acc[i] + k4_acc[i]);
    }

    // 最后调用一次前向动力学，更新加速度以供下一步使用
    forward_dynamics(model, state);
}

/// 半隐式欧拉积分器（备选的简单方法）
///
/// ## 算法
///
/// ```text
/// q̇_{n+1} = q̇_n + h * q̈_n
/// q_{n+1} = q_n + h * q̇_{n+1}
/// ```
///
/// 先更新速度，再用新速度更新位置。这种方法比显式欧拉法稳定，
/// 但精度低于RK4。
///
/// ## 适用场景
///
/// - 调试和快速原型
/// - 对精度要求不高的情况
/// - 需要更快计算速度的场合（每步只需1次动力学计算 vs RK4的4次）
///
#[allow(dead_code)]
pub fn semi_implicit_euler_step(model: &mut MultiBodyModel, state: &mut SimulationState, dt: f32) {
    forward_dynamics(model, state);

    for i in 0..model.nq {
        state.qvel[i] += dt * state.qacc[i];
        state.q[i] += dt * state.qvel[i];
    }

    forward_dynamics(model, state);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multibody::model::{HingeJoint, RigidBody};
    use bevy::math::{Mat3, Vec3};

    #[test]
    fn test_rk4_integration() {
        // 创建简单的单摆系统
        let mut model = MultiBodyModel::new();

        let mass = 1.0;
        let inertia = Mat3::from_diagonal(Vec3::new(0.1, 0.1, 0.1));
        let body = RigidBody::new(mass, inertia);
        let idx = model.add_body(body);

        let joint = HingeJoint {
            parent_body: -1,
            child_body: idx,
            axis: Vec3::X,
            body_offset: Vec3::new(0.0, -0.5, 0.0),
            joint_offset: Vec3::new(0.0, 0.5, 0.0),
            damping: 0.0,
            armature: 0.0,
            ..Default::default()
        };
        model.add_hinge_joint(joint);

        // 初始状态：从水平位置释放
        let mut state = SimulationState::new(model.nq);
        state.q[0] = std::f32::consts::PI / 2.0; // 90度（水平）
        state.qvel[0] = 0.0;

        // 积分几步
        let dt = 0.001;
        for _ in 0..10 {
            rk4_step(&mut model, &mut state, dt);
        }

        println!("After 10 steps:");
        println!("  q = {:.6}", state.q[0]);
        println!("  qvel = {:.6}", state.qvel[0]);

        // 单摆应该开始向下摆动（角度减小，速度为负）
        assert!(state.q[0] < std::f32::consts::PI / 2.0);
        assert!(state.qvel[0] < 0.0);
    }
}
