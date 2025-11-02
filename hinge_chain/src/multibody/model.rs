use bevy::math::{Mat3, Quat, Vec3};

/// 空间速度 (Spatial Velocity) - 6D 向量
/// 用于表示刚体在 6D 空间中的速度
#[derive(Debug, Clone, Copy)]
pub struct SpatialVelocity {
    /// 线速度分量 (3D)
    pub linear: Vec3,
    /// 角速度分量 (3D)
    pub angular: Vec3,
}

impl SpatialVelocity {
    pub fn zero() -> Self {
        Self {
            linear: Vec3::ZERO,
            angular: Vec3::ZERO,
        }
    }
}

/// 空间力 (Spatial Force) - 6D 向量
/// 用于表示刚体受到的力和力矩
#[derive(Debug, Clone, Copy)]
pub struct SpatialForce {
    /// 力分量 (3D)
    pub force: Vec3,
    /// 力矩分量 (3D)
    pub torque: Vec3,
}

impl SpatialForce {
    pub fn zero() -> Self {
        Self {
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
        }
    }
}

/// 空间惯性 (Spatial Inertia)
/// 表示刚体在 6D 空间中的惯性属性
///
/// 在 MuJoCo 中用 10 元素紧凑格式存储：
/// [mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, 0, 0, 0]
///
/// 本实现使用更直观的格式
#[derive(Debug, Clone)]
pub struct SpatialInertia {
    /// 质量 (kg)
    pub mass: f32,
    /// 3x3 旋转惯性张量 (kg*m^2)
    pub inertia: Mat3,
    /// 质心位置（相对于 body 原点）
    pub com_offset: Vec3,
}

impl SpatialInertia {
    /// 创建零惯性
    pub fn zero() -> Self {
        Self {
            mass: 0.0,
            inertia: Mat3::ZERO,
            com_offset: Vec3::ZERO,
        }
    }

    /// 从刚体创建空间惯性
    pub fn from_body(body: &RigidBody) -> Self {
        Self {
            mass: body.mass,
            inertia: body.inertia,
            com_offset: Vec3::ZERO, // 假设质心在 body 原点
        }
    }

    /// 空间惯性向量乘法：I * v = f
    /// 将空间速度映射到空间力
    ///
    /// 对应 MuJoCo 的 mju_mulInertVec
    pub fn mul_velocity(&self, v: SpatialVelocity) -> SpatialForce {
        // f_linear = m * v_linear + m * (com_offset × v_angular)
        let f_linear = self.mass * v.linear + self.mass * self.com_offset.cross(v.angular);

        // τ = I * ω + (m * com_offset) × v_linear
        let torque = self.inertia * v.angular + (self.mass * self.com_offset).cross(v.linear);

        SpatialForce {
            force: f_linear,
            torque,
        }
    }

    /// 空间惯性加法（用于 CRBA 的累积步骤）
    pub fn add(&self, other: &SpatialInertia) -> SpatialInertia {
        // 简化版本：假设两个惯性在同一参考系
        SpatialInertia {
            mass: self.mass + other.mass,
            inertia: self.inertia + other.inertia,
            com_offset: if self.mass + other.mass > 1e-10 {
                (self.mass * self.com_offset + other.mass * other.com_offset)
                    / (self.mass + other.mass)
            } else {
                Vec3::ZERO
            },
        }
    }
}

impl std::ops::AddAssign for SpatialInertia {
    fn add_assign(&mut self, other: Self) {
        *self = self.add(&other);
    }
}

/// Represents a rigid body in the multibody system
#[derive(Debug, Clone)]
pub struct RigidBody {
    /// Mass of the body (kg)
    pub mass: f32,
    /// Inertia tensor in body frame (kg*m^2)
    pub inertia: Mat3,
    /// Position in world frame
    pub position: Vec3,
    /// Orientation as quaternion
    pub orientation: Quat,
    /// Linear velocity
    pub velocity: Vec3,
    /// Angular velocity
    pub angular_velocity: Vec3,
    /// Spatial velocity (6D: [angular, linear]) at COM
    /// 对应MuJoCo的 cvel
    pub spatial_velocity: [f32; 6],
}

impl RigidBody {
    pub fn new(mass: f32, inertia: Mat3) -> Self {
        Self {
            mass,
            inertia,
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            spatial_velocity: [0.0; 6],
        }
    }
}

/// Represents a hinge joint connecting two bodies
#[derive(Debug, Clone)]
pub struct HingeJoint {
    /// Index of parent body (-1 for world/anchor)
    pub parent_body: i32,
    /// Index of child body
    pub child_body: usize,
    /// Joint axis in body frame (unit vector)
    pub axis: Vec3,
    /// Body offset: child body position relative to parent body (in parent frame)
    pub body_offset: Vec3,
    /// Joint offset: joint position relative to child body (in body frame)
    pub joint_offset: Vec3,
    /// Damping coefficient
    pub damping: f32,
    /// Armature (rotor inertia)
    pub armature: f32,
    /// Motion subspace (6D: [angular, linear])
    /// 对应MuJoCo的 cdof
    pub cdof: [f32; 6],
    /// Time derivative of motion subspace
    /// 对应MuJoCo的 cdof_dot
    pub cdof_dot: [f32; 6],
}

/// Complete multibody model
#[derive(Debug)]
pub struct MultiBodyModel {
    /// All rigid bodies in the system
    pub bodies: Vec<RigidBody>,
    /// All hinge joints
    pub joints: Vec<HingeJoint>,
    /// Number of degrees of freedom (= number of joints)
    pub nq: usize,
}

/// Simulation state using generalized coordinates
#[derive(Debug, Clone)]
pub struct SimulationState {
    /// Generalized positions (joint angles in radians)
    pub q: Vec<f32>,
    /// Generalized velocities (joint angular velocities)
    pub qvel: Vec<f32>,
    /// Generalized accelerations
    pub qacc: Vec<f32>,
    /// Generalized forces
    pub qfrc: Vec<f32>,
}

impl SimulationState {
    pub fn new(nq: usize) -> Self {
        Self {
            q: vec![0.0; nq],
            qvel: vec![0.0; nq],
            qacc: vec![0.0; nq],
            qfrc: vec![0.0; nq],
        }
    }
}

impl MultiBodyModel {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            joints: Vec::new(),
            nq: 0,
        }
    }

    /// Add a rigid body to the model
    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.push(body);
        self.bodies.len() - 1
    }

    /// Add a hinge joint to the model
    pub fn add_hinge_joint(&mut self, joint: HingeJoint) {
        self.joints.push(joint);
        self.nq += 1;
    }
}
