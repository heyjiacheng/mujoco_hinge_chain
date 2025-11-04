//! # 速度计算模块
//!
//! 计算空间速度和运动子空间的时间导数
//!
//! 对应MuJoCo的 mj_comVel (engine_core_smooth.c:1932-1997)

use super::model::{MultiBodyModel, SimulationState};
use super::spatial_algebra::{cross_motion, SpatialMotion};
use super::subtree_com::compute_subtree_com;
use bevy::math::Vec3;

/// 计算空间速度和cdof_dot
///
/// 对应MuJoCo的 mj_comVel() 函数
///
/// ## 算法
///
/// 前向传播，从根到叶计算每个body的空间速度：
/// 1. 初始化世界body的速度为0
/// 2. 对每个body：
///    - 继承父body的速度
///    - 对每个DOF：计算cdof_dot = cvel × cdof
///    - 更新cvel += cdof * qvel
///
/// ## 参考
/// - MuJoCo: engine_core_smooth.c:1932-1997
/// - QUICK_REFERENCE.md: Section "mj_comVel() Flow"
///
pub fn compute_velocities(model: &mut MultiBodyModel, state: &SimulationState) {
    // 初始化：世界body速度为0
    // (对应 MuJoCo line 1936: mju_zero(d->cvel, 6))

    // ⭐关键修复⭐: 计算子树质心
    // 对应 MuJoCo engine_core_smooth.c:184-202
    let subtree_com = compute_subtree_com(model);

    for (i, joint) in model.joints.iter_mut().enumerate() {
        let qvel = state.qvel[i];

        // 获取父body的空间速度
        let parent_cvel = if joint.parent_body >= 0 {
            let parent_idx = joint.parent_body as usize;
            SpatialMotion::from_array(&model.bodies[parent_idx].spatial_velocity)
        } else {
            SpatialMotion::zero()  // 世界body速度为0
        };

        // 计算当前关节的运动子空间 (cdof)
        // 对于Hinge关节: cdof = [axis, axis × offset]
        //
        // 参考: MuJoCo mju_dofCom (engine_util_spatial.c:445-457)
        //       MuJoCo mj_comPos (engine_core_smooth.c:223)
        //
        // ⭐关键修复⭐: offset方向
        // offset必须是从joint anchor到COM的向量（不是反过来！）
        // 物理原理: v_COM = ω × r，其中r是从旋转中心到质点的向量
        let child_body = &model.bodies[joint.child_body];

        // 使用父body的orientation来转换轴（对于根节点用世界坐标）
        let axis = if joint.parent_body >= 0 {
            let parent_body = &model.bodies[joint.parent_body as usize];
            parent_body.orientation * joint.axis
        } else {
            joint.axis
        };

        // offset = 从joint anchor到subtree COM的向量
        // joint_anchor = body.position + body.orientation * joint_offset
        // ⭐关键修复⭐: 使用subtree_com而不是body.position
        // 对应 MuJoCo engine_core_smooth.c:223:
        //   mju_sub3(offset, d->subtree_com+3*m->body_rootid[bi], d->xanchor+3*j);
        let joint_anchor = child_body.position + child_body.orientation * joint.joint_offset;
        let offset = subtree_com[joint.child_body] - joint_anchor;

        // cdof = [axis, axis × offset]
        // 根据右手定则，axis × offset给出线速度方向
        let cdof_angular = axis;
        let cdof_linear = axis.cross(offset);
        let cdof = SpatialMotion::new(cdof_angular, cdof_linear);

        // 计算 cdof_dot = cvel_parent × cdof (Lie bracket)
        //
        // 参考: MuJoCo line 1985
        //   mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));
        //
        // 这是科里奥利/离心力的来源！
        let cdof_dot_spatial = cross_motion(&parent_cvel, &cdof);

        // 更新body的空间速度
        // cvel = cvel_parent + cdof * qvel
        //
        // 参考: MuJoCo line 1988
        //   mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
        //   mju_addTo(cvel, tmp, 6);
        let cvel = parent_cvel.add(&cdof.scale(qvel));

        // 存储结果
        joint.cdof = cdof.to_array();
        joint.cdof_dot = cdof_dot_spatial.to_array();
        model.bodies[joint.child_body].spatial_velocity = cvel.to_array();

        // 同时更新简单的3D速度（用于可视化）
        model.bodies[joint.child_body].angular_velocity = Vec3::new(cvel.angular.x, cvel.angular.y, cvel.angular.z);
        model.bodies[joint.child_body].velocity = Vec3::new(cvel.linear.x, cvel.linear.y, cvel.linear.z);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multibody::{
        geometry::capsule_inertia,
        model::{HingeJoint, RigidBody},
    };

    #[test]
    fn test_compute_velocities() {
        // 创建简单的单关节系统
        let mut model = MultiBodyModel::new();

        let (mass, inertia) = capsule_inertia(0.5, 0.05, 1000.0);
        let body = RigidBody::new(mass, inertia);
        let idx = model.add_body(body);

        let joint = HingeJoint {
            parent_body: -1,
            child_body: idx,
            axis: Vec3::X,
            body_offset: Vec3::new(0.0, 0.0, -0.25),
            joint_offset: Vec3::new(0.0, 0.0, 0.25),
            damping: 0.1,
            armature: 0.01,
            cdof: [0.0; 6],
            cdof_dot: [0.0; 6],
        };
        model.add_hinge_joint(joint);

        // 设置关节速度
        let mut state = SimulationState::new(model.nq);
        state.qvel[0] = 1.0;  // 1 rad/s

        // 计算速度
        compute_velocities(&mut model, &state);

        // 验证：body应该有角速度
        let cvel = &model.bodies[0].spatial_velocity;
        println!("cvel: {:?}", cvel);

        // 角速度应该约等于 [1, 0, 0] (沿X轴)
        assert!((cvel[0] - 1.0).abs() < 0.1);
    }
}
