//! # 运动学模块
//!
//! 实现前向运动学算法，从广义坐标计算刚体的位置和姿态

use super::model::{MultiBodyModel, SimulationState};
use bevy::math::{Quat, Vec3};

/// 前向运动学：从广义坐标计算刚体位置和姿态
///
/// 这是多体动力学的核心算法之一，对应MuJoCo的`mj_kinematics()`函数。
///
/// ## 算法流程
///
/// 对于树形多体系统，从根节点到叶节点递归计算：
///
/// 1. **Body位置计算**:
///    ```text
///    p_body = p_parent + q_parent * body_offset
///    ```
///    其中 `q_parent` 是父body的姿态（四元数），`*` 表示四元数旋转向量
///
/// 2. **Joint旋转计算**:
///    ```text
///    q_joint = AxisAngle(axis, θ)
///    ```
///    从关节轴和广义坐标 θ 构造旋转四元数
///
/// 3. **Body姿态计算**:
///    ```text
///    q_body = q_parent * q_joint
///    ```
///    四元数乘法组合父body姿态和关节旋转
///
/// 4. **速度计算**:
///    ```text
///    v_body = v_parent + ω_parent × r + joint_axis * q̇
///    ω_body = ω_parent + joint_axis * q̇
///    ```
///
/// ## 坐标系说明
///
/// - **世界坐标系**: 固定的惯性参考系
/// - **Body坐标系**: 随刚体运动的局部坐标系
/// - **Joint坐标系**: Joint在body局部坐标系中的位置和方向
///
/// ## 参数
/// - `model`: 多体模型（包含所有body和joint的定义）
/// - `state`: 仿真状态（包含广义坐标 q 和广义速度 qvel）
///
/// ## 副作用
/// 更新 `model.bodies` 中每个body的 `position`, `orientation`, `velocity`, `angular_velocity`
///
pub fn forward_kinematics(model: &mut MultiBodyModel, state: &SimulationState) {
    // 使用临时数组避免借用冲突
    let mut positions = Vec::with_capacity(model.bodies.len());
    let mut orientations = Vec::with_capacity(model.bodies.len());
    let mut velocities = Vec::with_capacity(model.bodies.len());
    let mut angular_velocities = Vec::with_capacity(model.bodies.len());

    for (i, joint) in model.joints.iter().enumerate() {
        let angle = state.q[i];
        let angular_vel = state.qvel[i];

        // 获取父body的变换和速度
        let (parent_pos, parent_quat, parent_vel, parent_omega) = if joint.parent_body >= 0 {
            let parent_idx = joint.parent_body as usize;
            (
                positions[parent_idx],
                orientations[parent_idx],
                velocities[parent_idx],
                angular_velocities[parent_idx],
            )
        } else {
            // 根节点：相对于世界原点
            (Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO)
        };

        // === 位置和姿态计算 ===
        // 参考: MuJoCo engine_core_smooth.c:87-137

        // 步骤1: 首先继承父body的姿态和位置
        // 这是body的初始变换（在joint.body_offset处）
        let mut xpos = parent_pos + parent_quat * joint.body_offset;
        let mut xquat = parent_quat;

        // 步骤2: 计算关节anchor在世界坐标系中的位置
        // anchor = body当前位置 + body当前姿态 * joint在body局部的offset
        // 这个位置在关节旋转过程中应该保持固定
        let xanchor = xpos + xquat * joint.joint_offset;

        // 步骤3: 将关节轴从父body坐标系转换到世界坐标系
        let axis_world = if joint.parent_body >= 0 {
            parent_quat * joint.axis
        } else {
            joint.axis
        };

        // 步骤4: 在世界坐标系中构造关节旋转
        // 使用轴角表示：旋转角度 = angle，旋转轴 = axis_world
        let joint_quat = Quat::from_axis_angle(axis_world, angle);

        // 步骤5: 应用关节旋转到body姿态
        // body_quat = xquat * joint_quat
        xquat = xquat * joint_quat;

        // 步骤6: ⭐关键⭐ Off-Center Rotation Correction
        // 旋转后，joint_offset在世界坐标系中的位置变了
        // 我们需要调整body位置，使anchor保持在原位置
        //
        // 参考: MuJoCo engine_core_smooth.c:133-137
        //   mju_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);
        //   mju_sub3(xpos, xanchor, vec);
        //
        // 数学: xpos = xanchor - R(xquat) * joint_offset
        let joint_offset_rotated = xquat * joint.joint_offset;
        xpos = xanchor - joint_offset_rotated;

        // 步骤7: 规范化四元数，防止累积误差
        // 参考: MuJoCo engine_core_smooth.c:151
        //   mju_normalize4(xquat);
        //
        // 这对长时间仿真的数值稳定性至关重要
        let body_orientation = xquat.normalize();
        let body_position = xpos;

        // === 速度计算 ===

        // 从父body到当前body质心的向量
        let r = body_position - parent_pos;

        // 步骤4: 计算body的角速度
        // ω_body = ω_parent + axis_world * q̇
        let body_angular_velocity = parent_omega + axis_world * angular_vel;

        // 步骤5: 计算body的线速度
        // v_body = v_parent + ω_parent × r + v_joint
        // 其中 v_joint = ω_joint × r_joint (对于hinge关节，平移速度由旋转产生)
        let body_velocity = parent_vel + parent_omega.cross(r);

        positions.push(body_position);
        orientations.push(body_orientation);
        velocities.push(body_velocity);
        angular_velocities.push(body_angular_velocity);
    }

    // 应用计算结果到model
    for (i, body) in model.bodies.iter_mut().enumerate() {
        body.position = positions[i];
        body.orientation = orientations[i];
        body.velocity = velocities[i];
        body.angular_velocity = angular_velocities[i];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multibody::model::{HingeJoint, RigidBody};
    use bevy::math::Mat3;

    #[test]
    fn test_forward_kinematics() {
        // 创建简单的两连杆系统
        let mut model = MultiBodyModel::new();

        // 第一个body
        let body1 = RigidBody::new(1.0, Mat3::IDENTITY);
        let idx1 = model.add_body(body1);

        let joint1 = HingeJoint {
            parent_body: -1,
            child_body: idx1,
            axis: Vec3::X,
            body_offset: Vec3::new(0.0, -0.5, 0.0),
            joint_offset: Vec3::new(0.0, 0.5, 0.0),
            damping: 0.0,
            armature: 0.0,
            ..Default::default()
        };
        model.add_hinge_joint(joint1);

        // 初始化状态：第一个关节旋转45度
        let mut state = SimulationState::new(model.nq);
        state.q[0] = std::f32::consts::PI / 4.0; // 45度

        // 执行前向运动学
        forward_kinematics(&mut model, &state);

        // 检查结果
        let body = &model.bodies[0];
        println!("Body position: {:?}", body.position);
        println!("Body orientation: {:?}", body.orientation);

        // Body应该在(0, -0.5, 0)位置
        assert!((body.position.y + 0.5).abs() < 1e-5);
    }
}
