//! # 动力学模块
//!
//! 实现多体动力学核心算法：
//! - RNE (Recursive Newton-Euler Algorithm): 计算广义力
//! - CRBA (Composite Rigid Body Algorithm): 计算质量矩阵

use super::model::{MultiBodyModel, SimulationState};
use super::GRAVITY;
use bevy::math::{Mat3, Vec3};

/// 计算科里奥利力和离心力（qfrc_bias）
///
/// ## 理论背景
///
/// 科里奥利和离心力项来自于系统的非线性动力学：
/// ```text
/// M(q) * q̈ + C(q, q̇) * q̇ = τ
/// ```
///
/// 其中 C(q, q̇) * q̇ 就是科里奥利和离心力，在MuJoCo中称为 qfrc_bias。
///
/// 对于旋转刚体，主要的科里奥利项是：
/// ```text
/// ω × (I * ω) - 陀螺力矩（在body坐标系中）
/// ```
///
/// ## 简化实现
///
/// 本实现计算主要的陀螺效应。对于hinge关节链，每个body绕单轴旋转，
/// 陀螺力矩相对较小。完整的RNE算法会自然包含所有速度乘积项。
///
/// ## 参数
/// - `model`: 多体模型
/// - `qfrc_bias`: 输出的科里奥利和离心力
///
pub fn compute_coriolis_centrifugal(model: &MultiBodyModel, qfrc_bias: &mut [f32]) {
    qfrc_bias.fill(0.0);

    // 对每个关节，计算科里奥利和离心力的贡献
    for (i, joint) in model.joints.iter().enumerate() {
        let child_body = &model.bodies[joint.child_body];

        // 获取body的角速度（世界坐标系）
        let omega = child_body.angular_velocity;

        // 计算陀螺力矩: τ_gyro = ω × (I_world * ω)
        // 先将惯性张量转换到世界坐标系
        let rot_mat = bevy::math::Mat3::from_quat(child_body.orientation);
        let inertia_world = rot_mat * child_body.inertia * rot_mat.transpose();
        let angular_momentum = inertia_world * omega;
        let gyroscopic_torque = omega.cross(angular_momentum);

        // 关节轴在世界坐标系中的方向
        let axis = if joint.parent_body >= 0 {
            let parent_body = &model.bodies[joint.parent_body as usize];
            parent_body.orientation * joint.axis
        } else {
            joint.axis
        };

        // 投影到关节轴：qfrc_bias[i] = -gyroscopic_torque · axis
        // 负号是因为 qfrc_bias 在 MuJoCo 中定义为需要被抵消的力
        qfrc_bias[i] = -gyroscopic_torque.dot(axis);
    }
}

/// 计算广义力（使用简化的RNE算法）
///
/// ## 理论背景
///
/// 广义力 τ 是通过虚功原理从实际力映射得到的：
/// ```text
/// δW = τ · δq = F · δx
/// ```
///
/// 对于重力，广义力为：
/// ```text
/// τ = J^T * F_gravity
/// ```
///
/// 其中 J 是雅可比矩阵，对于旋转关节：
/// ```text
/// J_i = axis_i × r_i
/// ```
///
/// ## MuJoCo的RNE算法
///
/// 完整的RNE算法包括两个递归过程：
/// 1. **前向递归**: 从根到叶计算速度和加速度
/// 2. **后向递归**: 从叶到根计算力和力矩
///
/// 本实现使用简化版本：直接计算重力和阻尼的广义力
///
/// ## 关键点
///
/// 每个关节i必须考虑**所有下游body**的重力贡献，因为关节i的运动
/// 会带动所有子树运动。这是多体系统的核心特征。
///
/// ## 参数
/// - `model`: 多体模型
/// - `state`: 当前状态（需要速度信息用于计算阻尼）
/// - `qfrc_out`: 输出的广义力数组
///
pub fn compute_generalized_forces(
    model: &MultiBodyModel,
    state: &SimulationState,
    qfrc_out: &mut [f32],
) {
    qfrc_out.fill(0.0);

    // 计算科里奥利和离心力 (qfrc_bias)
    let mut qfrc_bias = vec![0.0; model.nq];
    compute_coriolis_centrifugal(model, &mut qfrc_bias);

    // 对每个关节，累积所有下游body的重力力矩
    for (i, joint) in model.joints.iter().enumerate() {
        let child_body = &model.bodies[joint.child_body];

        // 计算关节在世界坐标系中的位置
        // joint_pos = body_pos + body_quat * joint_offset
        let joint_pos = child_body.position + child_body.orientation * joint.joint_offset;

        // 计算关节轴在世界坐标系中的方向
        // 注意：轴是在父body坐标系中定义的
        let axis = if joint.parent_body >= 0 {
            let parent_body = &model.bodies[joint.parent_body as usize];
            parent_body.orientation * joint.axis
        } else {
            joint.axis
        };

        // 累积重力力矩：τ_i = Σ(j>=i) (r_j × F_gravity_j) · axis_i
        //
        // 物理解释：
        // - 关节i要支撑所有下游body的重量
        // - r_j 是从关节i到body j质心的向量
        // - 力矩投影到关节轴上得到广义力
        for j in i..model.joints.len() {
            let downstream_body = &model.bodies[model.joints[j].child_body];

            // 从关节到body质心的向量
            let r = downstream_body.position - joint_pos;

            // 重力 = m * g
            let gravity_force = GRAVITY * downstream_body.mass;

            // 力矩 = r × F
            let torque = r.cross(gravity_force);

            // 投影到关节轴：τ = torque · axis
            qfrc_out[i] += torque.dot(axis);
        }

        // 添加阻尼力：τ_damping = -damping * q̇
        // 阻尼模拟摩擦和能量耗散
        qfrc_out[i] -= joint.damping * state.qvel[i];

        // 减去科里奥利和离心力（这些是需要被抵消的力）
        // 总的广义力 = 重力 - 阻尼 - qfrc_bias
        qfrc_out[i] -= qfrc_bias[i];
    }
}

/// 计算质量矩阵（使用完整的 CRBA 算法）
///
/// ## 理论背景
///
/// 质量矩阵 M(q) 出现在运动方程中：
/// ```text
/// M(q) * q̈ + C(q,q̇) * q̇ = τ
/// ```
///
/// 其中：
/// - M(q): 质量矩阵（正定对称矩阵）
/// - C(q,q̇): 科里奥利和离心力矩阵
/// - τ: 广义力
///
/// ## CRBA (Composite Rigid Body Algorithm)
///
/// CRBA的核心思想：
/// - 将每个body及其所有下游body视为一个"复合刚体"
/// - 计算这个复合刚体相对于当前关节的惯性
///
/// ### 算法步骤（对应 MuJoCo 的 mj_crb）
///
/// 1. **初始化复合惯性**: crb[i] = 每个body的空间惯性
/// 2. **后向递推**: 从叶到根累积子树惯性
/// 3. **前向递推**: 对每个dof i，计算与其祖先的耦合 M[i,j]
///
/// ## 质量矩阵性质
///
/// - 对称: M[i,j] = M[j,i]
/// - 正定: 对任意 x≠0, x^T M x > 0
/// - 配置相关: M = M(q)
///
/// ## 参数
/// - `model`: 多体模型（需要先调用 forward_kinematics）
///
/// ## 返回值
/// 完整的质量矩阵 M (nq × nq，存储为 Vec<Vec<f32>>)
///
pub fn compute_mass_matrix(model: &MultiBodyModel) -> Vec<Vec<f32>> {
    use super::model::SpatialInertia;

    let nq = model.nq;
    let mut M = vec![vec![0.0; nq]; nq];

    // ===== 步骤 1: 初始化复合刚体惯性 =====
    let mut crb: Vec<SpatialInertia> = model
        .bodies
        .iter()
        .map(|body| SpatialInertia::from_body(body))
        .collect();

    // ===== 步骤 2: 后向递推 - 累积子树惯性 =====
    // 从叶到根：crb[parent] += crb[child]
    for i in (0..model.joints.len()).rev() {
        let joint = &model.joints[i];
        if joint.parent_body >= 0 {
            let child_crb = crb[joint.child_body].clone();
            crb[joint.parent_body as usize] += child_crb;
        }
    }

    // ===== 步骤 3: 前向递推 - 计算质量矩阵元素 =====
    for i in 0..nq {
        let joint_i = &model.joints[i];
        let body_i = &model.bodies[joint_i.child_body];

        // 计算关节 i 的位置和轴（世界坐标系）
        let joint_i_pos = body_i.position + body_i.orientation * joint_i.joint_offset;
        let axis_i = if joint_i.parent_body >= 0 {
            let parent = &model.bodies[joint_i.parent_body as usize];
            parent.orientation * joint_i.axis
        } else {
            joint_i.axis
        };

        // 计算从关节i的复合惯性投影
        // buf = crb[body_i] 在关节轴方向上的"有效惯量"
        let crb_i = &crb[joint_i.child_body];

        // 对关节 i 的每个祖先关节 j（包括自己）计算 M[i,j]
        let mut j_idx = i;
        loop {
            let joint_j = &model.joints[j_idx];
            let body_j = &model.bodies[joint_j.child_body];

            // 计算关节 j 的轴（世界坐标系）
            let axis_j = if joint_j.parent_body >= 0 {
                let parent = &model.bodies[joint_j.parent_body as usize];
                parent.orientation * joint_j.axis
            } else {
                joint_j.axis
            };

            // 计算 M[i,j] = M[j,i]（对称）
            // 这是通过两个关节轴对复合惯性的双重投影
            let m_ij = compute_mass_matrix_element(crb_i, &axis_i, &axis_j, body_i.position);

            M[i][j_idx] = m_ij;
            M[j_idx][i] = m_ij; // 对称性

            // 移动到父关节
            if joint_j.parent_body < 0 {
                break;
            }

            // 找到父关节的索引
            j_idx = model
                .joints
                .iter()
                .position(|j| j.child_body == joint_j.parent_body as usize)
                .unwrap_or(usize::MAX);

            if j_idx == usize::MAX {
                break;
            }
        }

        // 添加电机转子惯量到对角线
        M[i][i] += joint_i.armature;
    }

    M
}

/// 计算质量矩阵的单个元素 M[i,j]
///
/// 基于复合刚体惯性和两个关节轴的投影
///
/// ## 参数
/// - `crb`: 复合刚体惯性
/// - `axis_i`: 关节 i 的轴（世界坐标系）
/// - `axis_j`: 关节 j 的轴（世界坐标系）
/// - `body_pos`: body 的位置（用于计算质心效应）
///
fn compute_mass_matrix_element(
    crb: &super::model::SpatialInertia,
    axis_i: &bevy::math::Vec3,
    axis_j: &bevy::math::Vec3,
    body_pos: bevy::math::Vec3,
) -> f32 {
    // 对于Hinge关节，运动自由度是纯旋转
    // cdof = [0, 0, 0, axis_x, axis_y, axis_z] （6维空间向量）
    //
    // 质量矩阵元素：M[i,j] = cdof_j^T * I_spatial * cdof_i
    //
    // 对于纯旋转关节，空间惯性矩阵的角动量部分贡献为：
    // M[i,j] = axis_j^T * I_rot * axis_i + m * (axis_j × com)^T * (axis_i × com)
    //
    // 第一项：旋转惯性
    // 第二项：质心偏移引起的平移-旋转耦合

    // 第一项：旋转惯性贡献
    let i_axis_i = crb.inertia * (*axis_i);
    let mut m_ij = axis_j.dot(i_axis_i);

    // 第二项：质心偏移的贡献
    // 对于Capsule，质心在body原点，所以com_offset ≈ 0
    // 但为了通用性，我们仍然计算这一项
    if crb.com_offset.length_squared() > 1e-10 {
        // 计算 axis × com_offset
        let r_i = axis_i.cross(crb.com_offset);
        let r_j = axis_j.cross(crb.com_offset);
        // m * (axis_j × com) · (axis_i × com)
        m_ij += crb.mass * r_j.dot(r_i);
    }

    m_ij
}

/// 求解线性系统 M * x = b（简单的高斯消元法）
///
/// ## 参数
/// - `M`: 质量矩阵 (nq × nq)
/// - `b`: 右端项（广义力）
///
/// ## 返回值
/// 解向量 x（广义加速度）
///
fn solve_linear_system(M: &Vec<Vec<f32>>, b: &[f32]) -> Vec<f32> {
    let n = b.len();

    // 创建增广矩阵 [M | b]
    let mut aug = vec![vec![0.0; n + 1]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = M[i][j];
        }
        aug[i][n] = b[i];
    }

    // 高斯消元（前向消元）
    for k in 0..n {
        // 找到主元
        let mut max_row = k;
        for i in (k + 1)..n {
            if aug[i][k].abs() > aug[max_row][k].abs() {
                max_row = i;
            }
        }

        // 交换行
        if max_row != k {
            aug.swap(k, max_row);
        }

        // 消元
        for i in (k + 1)..n {
            if aug[k][k].abs() > 1e-10 {
                let factor = aug[i][k] / aug[k][k];
                for j in k..=n {
                    aug[i][j] -= factor * aug[k][j];
                }
            }
        }
    }

    // 回代
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        x[i] = if aug[i][i].abs() > 1e-10 {
            sum / aug[i][i]
        } else {
            0.0
        };
    }

    x
}

/// 计算广义加速度：q̈ = M^(-1) * τ
///
/// ## 前向动力学
///
/// 这是前向动力学的最后一步：从广义力计算广义加速度。
///
/// 求解线性系统：
/// ```text
/// M(q) * q̈ = τ
/// ```
///
/// 使用高斯消元法求解（未来可优化为 LDL^T 分解）
///
/// ## 参数
/// - `model`: 多体模型
/// - `state`: 仿真状态（输入 qfrc，输出 qacc）
///
pub fn compute_acceleration(model: &MultiBodyModel, state: &mut SimulationState) {
    // 计算完整质量矩阵
    let mass_matrix = compute_mass_matrix(model);

    // 求解 M * qacc = qfrc
    let qacc = solve_linear_system(&mass_matrix, &state.qfrc);

    // 更新状态
    state.qacc.copy_from_slice(&qacc);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multibody::{
        kinematics::forward_kinematics,
        model::{HingeJoint, RigidBody},
    };
    use bevy::math::Mat3;

    #[test]
    fn test_compute_generalized_forces() {
        // 创建简单的单摆系统
        let mut model = MultiBodyModel::new();

        let mass = 1.0;
        let inertia = Mat3::IDENTITY;
        let body = RigidBody::new(mass, inertia);
        let idx = model.add_body(body);

        let joint = HingeJoint {
            parent_body: -1,
            child_body: idx,
            axis: Vec3::X,
            body_offset: Vec3::new(0.0, -0.5, 0.0),
            joint_offset: Vec3::new(0.0, 0.5, 0.0),
            damping: 0.1,
            armature: 0.0,
        };
        model.add_hinge_joint(joint);

        let mut state = SimulationState::new(model.nq);
        state.q[0] = 0.0; // 垂直向下
        state.qvel[0] = 0.0;

        // 计算运动学
        forward_kinematics(&mut model, &state);

        // 计算广义力
        let mut qfrc = vec![0.0; model.nq];
        compute_generalized_forces(&model, &state, &mut qfrc);

        println!("广义力: {:?}", qfrc);

        // 垂直向下时，重力力矩应该为0
        assert!(qfrc[0].abs() < 1e-5);
    }

    #[test]
    fn test_compute_mass_matrix() {
        // 测试质量矩阵计算
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
            armature: 0.01,
        };
        model.add_hinge_joint(joint);

        let mut state = SimulationState::new(model.nq);
        forward_kinematics(&mut model, &state);

        let M = compute_mass_matrix(&model);

        println!("质量矩阵对角元素: {:?}", M);

        // 质量矩阵应该为正
        assert!(M[0] > 0.0);
    }
}
