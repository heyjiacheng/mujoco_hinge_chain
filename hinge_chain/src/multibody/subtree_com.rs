//! # Subtree COM计算模块
//!
//! 计算每个body及其所有子body的总质心位置

use super::model::MultiBodyModel;
use bevy::math::Vec3;

/// 计算子树质心
///
/// 对于树形多体系统，每个body的子树质心包括：
/// - 该body自身的质量和位置
/// - 所有子body的质量和位置
///
/// ## 算法
///
/// 1. 初始化: subtree_com[i] = mass[i] * position[i]
/// 2. 后向累积(从叶到根): subtree_com[parent] += subtree_com[child]
/// 3. 归一化: subtree_com[i] /= subtree_mass[i]
///
/// ## 参数
/// - `model`: 多体模型（需要已经运行过forward_kinematics）
///
/// ## 返回
/// Vec<Vec3>: 每个body的子树质心位置（世界坐标系）
///
pub fn compute_subtree_com(model: &MultiBodyModel) -> Vec<Vec3> {
    let nbody = model.bodies.len();

    // 步骤1: 初始化 - 加权质心位置
    // subtree_com_weighted[i] = mass[i] * position[i]
    let mut subtree_com_weighted = Vec::with_capacity(nbody);
    let mut subtree_mass = Vec::with_capacity(nbody);

    for body in &model.bodies {
        subtree_com_weighted.push(body.position * body.mass);
        subtree_mass.push(body.mass);
    }

    // 步骤2: 后向累积 (从叶到根)
    // 对于链式系统，从最后一个body开始，向上累积到根
    for i in (0..model.joints.len()).rev() {
        let joint = &model.joints[i];

        if joint.parent_body >= 0 {
            let parent_idx = joint.parent_body as usize;
            let child_idx = joint.child_body;

            // 将child的子树累积到parent
            // 先读取child的值，避免借用冲突
            let child_com_weighted = subtree_com_weighted[child_idx];
            let child_mass = subtree_mass[child_idx];

            subtree_com_weighted[parent_idx] += child_com_weighted;
            subtree_mass[parent_idx] += child_mass;
        }
    }

    // 步骤3: 归一化 - 得到真正的质心位置
    // subtree_com[i] = subtree_com_weighted[i] / subtree_mass[i]
    let mut subtree_com = Vec::with_capacity(nbody);

    for i in 0..nbody {
        if subtree_mass[i] > 1e-10 {
            subtree_com.push(subtree_com_weighted[i] / subtree_mass[i]);
        } else {
            subtree_com.push(model.bodies[i].position);
        }
    }

    subtree_com
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multibody::{
        geometry::capsule_inertia,
        kinematics::forward_kinematics,
        model::{HingeJoint, RigidBody, SimulationState},
    };

    #[test]
    fn test_subtree_com_single_body() {
        // 单个body，子树质心应该等于自身位置
        let mut model = MultiBodyModel::new();

        let (mass, inertia) = capsule_inertia(0.5, 0.05, 1000.0);
        let mut body = RigidBody::new(mass, inertia);
        body.position = Vec3::new(0.0, 0.0, -1.0);

        let idx = model.add_body(body);

        let joint = HingeJoint {
            parent_body: -1,
            child_body: idx,
            axis: Vec3::X,
            body_offset: Vec3::new(0.0, 0.0, -1.0),
            joint_offset: Vec3::new(0.0, 0.0, 0.5),
            damping: 0.1,
            armature: 0.01,
            cdof: [0.0; 6],
            cdof_dot: [0.0; 6],
        };
        model.add_hinge_joint(joint);

        let state = SimulationState::new(model.nq);
        forward_kinematics(&mut model, &state);

        let subtree_com = compute_subtree_com(&model);

        // 单个body的子树质心应该等于自身位置
        let diff = (subtree_com[0] - model.bodies[0].position).length();
        assert!(diff < 1e-6, "Subtree COM should equal body position for single body");
    }

    #[test]
    fn test_subtree_com_two_bodies() {
        // 两个body，第一个的子树质心应该在两者之间
        let mut model = MultiBodyModel::new();

        let (mass, inertia) = capsule_inertia(0.5, 0.05, 1000.0);

        // Body 0
        let mut body0 = RigidBody::new(mass, inertia);
        body0.position = Vec3::new(0.0, 0.0, -0.5);
        let idx0 = model.add_body(body0);

        let joint0 = HingeJoint {
            parent_body: -1,
            child_body: idx0,
            axis: Vec3::X,
            body_offset: Vec3::new(0.0, 0.0, -0.5),
            joint_offset: Vec3::new(0.0, 0.0, 0.25),
            damping: 0.1,
            armature: 0.01,
            cdof: [0.0; 6],
            cdof_dot: [0.0; 6],
        };
        model.add_hinge_joint(joint0);

        // Body 1
        let mut body1 = RigidBody::new(mass, inertia);
        body1.position = Vec3::new(0.0, 0.0, -1.0);
        let idx1 = model.add_body(body1);

        let joint1 = HingeJoint {
            parent_body: 0,
            child_body: idx1,
            axis: Vec3::Y,
            body_offset: Vec3::new(0.0, 0.0, -0.5),
            joint_offset: Vec3::new(0.0, 0.0, 0.25),
            damping: 0.1,
            armature: 0.01,
            cdof: [0.0; 6],
            cdof_dot: [0.0; 6],
        };
        model.add_hinge_joint(joint1);

        let state = SimulationState::new(model.nq);
        forward_kinematics(&mut model, &state);

        let subtree_com = compute_subtree_com(&model);

        // Body 0的子树质心应该在-0.75附近（两个等质量body的平均位置）
        println!("Body 0 subtree COM: {:?}", subtree_com[0]);
        println!("Body 1 subtree COM: {:?}", subtree_com[1]);

        // 两个等质量body在-0.5和-1.0，平均应该在-0.75
        assert!((subtree_com[0].z + 0.75).abs() < 0.01, "Subtree COM incorrect");

        // Body 1的子树质心应该等于自身（它是叶节点）
        let diff = (subtree_com[1] - model.bodies[1].position).length();
        assert!(diff < 1e-6, "Leaf subtree COM should equal body position");
    }
}
