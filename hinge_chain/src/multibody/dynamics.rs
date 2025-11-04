//! # 动力学模块
//!
//! 实现多体动力学核心算法：
//! - RNE (Recursive Newton-Euler Algorithm): 计算广义力
//! - CRBA (Composite Rigid Body Algorithm): 计算质量矩阵

use super::model::{MultiBodyModel, SimulationState};
use super::spatial_algebra::{cross_force, inertia_mul_motion, SpatialForce, SpatialMotion};
use super::GRAVITY;

/// 计算广义力（使用完整的RNE算法，与MuJoCo一致）
///
/// ## 理论背景
///
/// 递归牛顿-欧拉算法（RNE）计算广义力 τ，包括：
/// - 重力
/// - 科里奥利力和离心力
/// - 被动力（弹簧、阻尼等）
///
/// ## MuJoCo的RNE算法（mj_rne, engine_core_smooth.c:2068-2119）
///
/// 完整的RNE算法包括两个递归过程：
/// 1. **前向递归**: 从根到叶计算加速度
/// 2. **后向递归**: 从叶到根计算力和力矩
/// 3. **投影**: 将空间力投影到关节空间
///
/// ## 算法流程
///
/// ### 前向传递（计算加速度）
/// ```text
/// for each body (from root to leaf):
///     // 科里奥利加速度项
///     cacc = cacc_parent + cdof_dot * qvel
///
///     // 如果计算真实加速度（flg_acc=1）
///     if flg_acc:
///         cacc += cdof * qacc
///
///     // 计算空间力
///     cfrc_body = cinert * cacc + cvel × (cinert * cvel)
/// ```
///
/// ### 后向传递（累积力）
/// ```text
/// for each body (from leaf to root):
///     cfrc_body[parent] += cfrc_body[child]
/// ```
///
/// ### 投影到关节空间
/// ```text
/// for each dof:
///     qfrc[i] = cdof[i]^T * cfrc_body[i]
/// ```
///
/// ## 参数
/// - `model`: 多体模型（需要已计算 cvel, cdof, cdof_dot）
/// - `state`: 当前状态（需要 qvel）
/// - `flg_acc`: 是否包含加速度项（0=只计算qfrc_bias，1=完整力）
/// - `qfrc_out`: 输出的广义力数组
///
pub fn compute_generalized_forces(
    model: &MultiBodyModel,
    state: &SimulationState,
    flg_acc: bool,
    qfrc_out: &mut [f32],
) {
    qfrc_out.fill(0.0);

    let nbody = model.bodies.len();

    // ===== 1. 初始化世界加速度 =====
    // 对应 MuJoCo line 2074-2077
    //
    // ⚠️ 关键区别：MuJoCo 计算 qfrc_bias，运动方程是 M*qacc = -qfrc_bias
    // 我们直接计算 qfrc，运动方程是 M*qacc = qfrc
    // 因此我们需要 qfrc = -qfrc_bias
    //
    // MuJoCo 设置 cacc = -gravity 来计算 qfrc_bias
    // 我们设置 cacc = +gravity 来计算 qfrc = -qfrc_bias
    //
    // ⚠️ 注意：在 MuJoCo 中，body[0] 是虚拟的"世界body"
    // 但在我们的实现中，bodies[0] 是第一个实际的刚体
    // 所以我们需要单独存储世界加速度
    let world_cacc = SpatialMotion::new(
        bevy::math::Vec3::ZERO,
        GRAVITY, // ⭐修复⭐ 使用 +gravity（而不是 MuJoCo 的 -gravity）
    );

    let mut cacc = vec![SpatialMotion::zero(); nbody];

    // ===== 2. 前向传递：累积加速度，计算力 =====
    // 对应 MuJoCo line 2080-2093
    let mut cfrc_body = vec![SpatialForce::zero(); nbody];

    for (i, joint) in model.joints.iter().enumerate() {
        let child_idx = joint.child_body;
        let child_body = &model.bodies[child_idx];

        // 获取父body的加速度
        // 如果 parent_body = -1，表示连接到世界，使用世界加速度
        let parent_cacc = if joint.parent_body >= 0 {
            cacc[joint.parent_body as usize]
        } else {
            world_cacc // 世界加速度（包含重力）
        };

        // 从 joint 获取 cdof 和 cdof_dot（6D向量）
        let cdof = SpatialMotion::from_array(&joint.cdof);
        let cdof_dot = SpatialMotion::from_array(&joint.cdof_dot);

        // ⭐核心⭐ 计算加速度：
        // cacc = cacc_parent + cdof_dot * qvel
        //
        // 对应 MuJoCo line 2085-2086:
        //   mju_mulDofVec(tmp, d->cdof_dot+6*bda, d->qvel+bda, ...)
        //   mju_add(loc_cacc+6*i, loc_cacc+6*parent, tmp, 6)
        let mut child_cacc = parent_cacc.add(&cdof_dot.scale(state.qvel[i]));

        // 如果需要真实加速度（不只是 qfrc_bias）
        // cacc += cdof * qacc
        //
        // 对应 MuJoCo line 2088-2091
        if flg_acc {
            child_cacc = child_cacc.add(&cdof.scale(state.qacc[i]));
        }

        cacc[child_idx] = child_cacc;

        // ⭐核心⭐ 计算空间力：
        // cfrc_body = cinert * cacc + cvel × (cinert * cvel)
        //
        // 对应 MuJoCo line 2095-2099:
        //   mju_mulInertVec(loc_cfrc_body+6*i, d->cinert+10*i, loc_cacc+6*i)
        //   mju_mulInertVec(tmp, d->cinert+10*i, d->cvel+6*i)
        //   mju_crossForce(tmp1, d->cvel+6*i, tmp)
        //   mju_addTo(loc_cfrc_body+6*i, tmp1, 6)

        // 第一项：I * cacc
        let f1 = inertia_mul_motion(
            child_body.mass,
            child_body.inertia,
            bevy::math::Vec3::ZERO, // com_offset = 0 for capsule
            &child_cacc,
        );

        // 第二项：cvel × (I * cvel) - 科里奥利/离心力
        let cvel = SpatialMotion::from_array(&child_body.spatial_velocity);
        let i_cvel = inertia_mul_motion(
            child_body.mass,
            child_body.inertia,
            bevy::math::Vec3::ZERO,
            &cvel,
        );
        let f2 = cross_force(&cvel, &i_cvel);

        cfrc_body[child_idx] = f1.add(&f2);
    }

    // ===== 3. 后向传递：累积子体力到父体 =====
    // 对应 MuJoCo line 2102-2106
    for i in (0..model.joints.len()).rev() {
        let joint = &model.joints[i];
        if joint.parent_body >= 0 {
            let child_force = cfrc_body[joint.child_body];
            cfrc_body[joint.parent_body as usize] += child_force;
        }
    }

    // ===== 4. 投影到关节空间：qfrc = cdof^T * cfrc_body =====
    // 对应 MuJoCo line 2109
    for (i, joint) in model.joints.iter().enumerate() {
        let cdof = SpatialMotion::from_array(&joint.cdof);
        let force = &cfrc_body[joint.child_body];

        // qfrc[i] = cdof[i] · cfrc_body[i]
        // = cdof.angular · force.torque + cdof.linear · force.force
        qfrc_out[i] = cdof.angular.dot(force.torque) + cdof.linear.dot(force.force);

        // ===== 5. 添加被动力（阻尼） =====
        // 对应 MuJoCo 的 mj_passive()
        qfrc_out[i] -= joint.damping * state.qvel[i];
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
    #[allow(non_snake_case)]
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

        // 计算关节轴（世界坐标系）
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
    _body_pos: bevy::math::Vec3,
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
/// - `m`: 质量矩阵 (nq × nq)
/// - `b`: 右端项（广义力）
///
/// ## 返回值
/// 解向量 x（广义加速度）
///
fn solve_linear_system(m: &Vec<Vec<f32>>, b: &[f32]) -> Vec<f32> {
    let n = b.len();

    // 创建增广矩阵 [M | b]
    let mut aug = vec![vec![0.0; n + 1]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = m[i][j];
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
        velocity::compute_velocities,
    };
    use bevy::math::{Mat3, Vec3};

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
            ..Default::default()
        };
        model.add_hinge_joint(joint);

        let mut state = SimulationState::new(model.nq);
        state.q[0] = 0.0; // 垂直向下
        state.qvel[0] = 0.0;

        // 计算运动学
        forward_kinematics(&mut model, &state);

        // 计算速度（需要 cdof 和 cdof_dot）
        compute_velocities(&mut model, &state);

        // 计算广义力
        let mut qfrc = vec![0.0; model.nq];
        compute_generalized_forces(&model, &state, false, &mut qfrc);

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
            ..Default::default()
        };
        model.add_hinge_joint(joint);

        let state = SimulationState::new(model.nq);
        forward_kinematics(&mut model, &state);

        let m = compute_mass_matrix(&model);

        println!("质量矩阵对角元素: {:?}", m[0][0]);

        // 质量矩阵对角元素应该为正
        assert!(m[0][0] > 0.0);
    }
}
