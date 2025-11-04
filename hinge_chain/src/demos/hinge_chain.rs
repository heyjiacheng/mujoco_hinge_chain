//! # Hinge Chain Demo
//!
//! 这是一个使用多体动力学框架实现的HingeChain演示程序。
//!
//! ## 系统描述
//!
//! - **6个Capsule刚体**：每个长0.5m，半径0.05m，密度1000 kg/m³
//! - **6个Hinge关节**：交替沿X轴和Y轴旋转
//! - **固定点**：第一个关节固定在世界坐标原点
//! - **重力**：9.81 m/s²沿-Z方向（与MuJoCo一致）
//!
//! ## 物理参数
//!
//! - 时间步长: 0.002s (500Hz)
//! - 积分器: RK4 (四阶龙格库塔)
//! - 关节阻尼: 0.1 N·m·s/rad
//! - 电机转子惯量: 0.01 kg·m²

use crate::multibody::*;
use bevy::math::Vec3;

/// Capsule几何参数
pub const CAPSULE_HALF_LENGTH: f32 = 0.25; // 半长（capsule总长0.5m）
pub const CAPSULE_RADIUS: f32 = 0.05;
pub const CAPSULE_DENSITY: f32 = 1000.0;

/// 关节参数
pub const JOINT_DAMPING: f32 = 0.1;
pub const JOINT_ARMATURE: f32 = 0.01;

/// Hinge Chain数量
pub const NUM_CAPSULES: usize = 6;

/// 创建HingeChain多体模型
///
/// ## 几何结构
///
/// ```text
///     ● (anchor point at origin)
///     |
///     ├─── Capsule 1 (center at z=-0.25)
///     |    Joint 1 (at z=0, axis=X)
///     |
///     ├─── Capsule 2 (center at z=-0.75)
///     |    Joint 2 (at z=-0.5, axis=Y)
///     |
///     ├─── Capsule 3 (center at z=-1.25)
///     |    Joint 3 (at z=-1.0, axis=X)
///     ...
/// ```
///
/// ## 坐标系说明（与MuJoCo一致）
///
/// - **世界坐标系**: Z轴向上为正，重力沿-Z方向
/// - **Body坐标系**: Capsule的质心在原点，长轴沿Z轴，从+0.25到-0.25
/// - **Joint坐标系**: Joint位于capsule的顶端（body坐标系中的z=+0.25处）
/// - **连接规则**: 每个capsule的底部（-0.25）连接到下一个capsule的顶部（+0.25）
///
/// ## 返回值
///
/// - `(model, state)`: 初始化好的模型和仿真状态
///
pub fn create_hinge_chain() -> (MultiBodyModel, SimulationState) {
    let mut model = MultiBodyModel::new();

    // 计算Capsule的质量和惯性
    let (mass, inertia) = capsule_inertia(
        2.0 * CAPSULE_HALF_LENGTH,
        CAPSULE_RADIUS,
        CAPSULE_DENSITY,
    );

    println!("=== Hinge Chain 配置 ===");
    println!("Capsule数量: {}", NUM_CAPSULES);
    println!("Capsule质量: {:.4} kg", mass);
    println!("Capsule惯性: {:.6?}", inertia);

    // 创建bodies和joints
    let mut parent_body: i32 = -1; // -1表示连接到世界/锚点

    for i in 0..NUM_CAPSULES {
        // 创建刚体
        let body = RigidBody::new(mass, inertia);
        let body_idx = model.add_body(body);

        // 关节轴：交替X和Y轴（与MuJoCo XML一致）
        // hinge1: axis="1 0 0" (X轴)
        // hinge2: axis="0 1 0" (Y轴)
        // ...
        let axis = if i % 2 == 0 {
            Vec3::X // 沿X轴旋转
        } else {
            Vec3::Y // 沿Y轴旋转
        };

        // Body offset: 相对父body的位置（与MuJoCo XML一致）
        // MuJoCo XML: <body name="capsuleN" pos="0 0 -0.5">
        // - 第一个body: 从世界原点向下0.25m（沿-Z）
        // - 后续body: 从父body质心向下0.5m（一个capsule长度）
        let body_offset = if i == 0 {
            Vec3::new(0.0, 0.0, -CAPSULE_HALF_LENGTH)
        } else {
            Vec3::new(0.0, 0.0, -2.0 * CAPSULE_HALF_LENGTH)
        };

        // Joint offset: 关节在body局部坐标系中的位置
        // MuJoCo XML: <joint pos="0 0 0.25" .../>
        // 所有关节都在capsule顶端（z=+0.25）
        let joint_offset = Vec3::new(0.0, 0.0, CAPSULE_HALF_LENGTH);

        let joint = HingeJoint {
            parent_body,
            child_body: body_idx,
            axis,
            body_offset,
            joint_offset,
            damping: JOINT_DAMPING,
            armature: JOINT_ARMATURE,
            cdof: [0.0; 6],      // Will be computed in forward_kinematics
            cdof_dot: [0.0; 6],  // Will be computed in compute_velocities
        };

        model.add_hinge_joint(joint);
        parent_body = body_idx as i32;
    }

    // 初始化状态，添加小扰动以打破对称性
    let mut state = SimulationState::new(model.nq);

    // 初始角度扰动（单位：弧度）
    // 这些扰动会激发系统的摆动
    state.q[0] = 0.9; // 约51.4度
    // state.q[1] = -0.9; // 约-51.4度
    // state.q[2] = 0.0;
    // if model.nq > 1 {
    //     state.q[2] = 0.0;
    // }

    println!("初始状态:");
    println!("  q = {:?}", state.q);
    println!("========================\n");

    (model, state)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_hinge_chain() {
        let (model, state) = create_hinge_chain();

        assert_eq!(model.nq, NUM_CAPSULES);
        assert_eq!(model.bodies.len(), NUM_CAPSULES);
        assert_eq!(model.joints.len(), NUM_CAPSULES);
        assert_eq!(state.q.len(), NUM_CAPSULES);

        // 检查第一个关节固定在世界
        assert_eq!(model.joints[0].parent_body, -1);

        // 检查链式连接
        for i in 1..NUM_CAPSULES {
            assert_eq!(model.joints[i].parent_body as usize, i - 1);
        }
    }
}
