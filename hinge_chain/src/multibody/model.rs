//! # 多体模型数据结构
//!
//! 定义多体动力学系统的核心数据结构

use bevy::math::{Mat3, Quat, Vec3};

/// 空间惯性 (Spatial Inertia)
///
/// 表示刚体的惯性属性，用于动力学计算
///
/// ## 物理意义
/// - 描述刚体抵抗运动变化的能力
/// - 包含质量（平动惯性）和转动惯量（转动惯性）
///
/// ## 存储格式
/// MuJoCo 使用紧凑的 10 元素格式: [mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, 0, 0, 0]
/// 本实现使用更直观的分离格式
#[derive(Debug, Clone)]
pub struct SpatialInertia {
    /// 质量 (kg)
    pub mass: f32,
    /// 3x3 转动惯量张量 (kg·m²)，在体坐标系中
    pub inertia: Mat3,
    /// 质心相对于 body 原点的偏移
    /// 对于质心在原点的物体（如胶囊），此值为零
    pub com_offset: Vec3,
}

impl SpatialInertia {
    /// 创建零惯性（用于初始化）
    pub fn zero() -> Self {
        Self {
            mass: 0.0,
            inertia: Mat3::ZERO,
            com_offset: Vec3::ZERO,
        }
    }

    /// 从刚体创建空间惯性
    ///
    /// 假设质心在 body 原点（对胶囊等对称物体成立）
    pub fn from_body(body: &RigidBody) -> Self {
        Self {
            mass: body.mass,
            inertia: body.inertia,
            com_offset: Vec3::ZERO,
        }
    }

    /// 累加两个空间惯性（用于 CRBA 算法）
    ///
    /// 计算复合刚体的惯性，假设两个惯性在同一参考系中
    pub fn add(&self, other: &SpatialInertia) -> SpatialInertia {
        let total_mass = self.mass + other.mass;
        SpatialInertia {
            mass: total_mass,
            inertia: self.inertia + other.inertia,
            com_offset: if total_mass > 1e-10 {
                (self.mass * self.com_offset + other.mass * other.com_offset) / total_mass
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

/// 刚体 (Rigid Body)
///
/// 表示多体系统中的一个刚性物体
///
/// ## 物理属性
/// - 质量和惯性张量定义了物体的惯性特性
/// - 位置和姿态描述物体在空间中的配置
/// - 速度描述物体的运动状态
#[derive(Debug, Clone)]
pub struct RigidBody {
    /// 质量 (kg)
    pub mass: f32,
    /// 转动惯量张量，在体坐标系中 (kg·m²)
    pub inertia: Mat3,

    /// 位置（世界坐标系）
    pub position: Vec3,
    /// 姿态（四元数表示）
    pub orientation: Quat,

    /// 线速度（世界坐标系）
    pub velocity: Vec3,
    /// 角速度（世界坐标系）
    pub angular_velocity: Vec3,

    /// 6D 空间速度 [ω_x, ω_y, ω_z, v_x, v_y, v_z]
    /// 对应 MuJoCo 的 cvel (spatial velocity at center of mass)
    pub spatial_velocity: [f32; 6],
}

impl RigidBody {
    /// 创建新的刚体
    ///
    /// # 参数
    /// - `mass`: 质量 (kg)
    /// - `inertia`: 转动惯量张量 (kg·m²)
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

/// 铰链关节 (Hinge Joint)
///
/// 连接两个刚体的单自由度旋转关节
///
/// ## 拓扑结构
/// - 父体可以是另一个刚体或世界坐标系（parent_body = -1）
/// - 子体必须是系统中的一个刚体
///
/// ## 运动约束
/// - 只允许绕指定轴旋转
/// - 约束掉平移和其他旋转自由度
#[derive(Debug, Clone)]
pub struct HingeJoint {
    /// 父体索引（-1 表示固定在世界坐标系）
    pub parent_body: i32,
    /// 子体索引
    pub child_body: usize,

    /// 关节轴方向（在父体坐标系中，单位向量）
    pub axis: Vec3,

    /// 子体相对父体的偏移（在父体坐标系中）
    pub body_offset: Vec3,
    /// 关节相对子体的偏移（在子体坐标系中）
    pub joint_offset: Vec3,

    /// 阻尼系数（模拟摩擦和能量耗散）
    pub damping: f32,
    /// 电机转子惯量（附加到关节的惯性）
    pub armature: f32,

    /// 运动子空间 6D: [ω_x, ω_y, ω_z, v_x, v_y, v_z]
    /// 对应 MuJoCo 的 cdof (constraint degrees of freedom)
    pub cdof: [f32; 6],
    /// 运动子空间的时间导数
    /// 对应 MuJoCo 的 cdof_dot
    pub cdof_dot: [f32; 6],
}

impl Default for HingeJoint {
    fn default() -> Self {
        Self {
            parent_body: -1,
            child_body: 0,
            axis: Vec3::X,
            body_offset: Vec3::ZERO,
            joint_offset: Vec3::ZERO,
            damping: 0.0,
            armature: 0.0,
            cdof: [0.0; 6],
            cdof_dot: [0.0; 6],
        }
    }
}

/// 多体动力学模型
///
/// 包含整个多体系统的拓扑结构和物理属性
///
/// ## 系统假设
/// - 树状拓扑结构（无闭环）
/// - 使用广义坐标描述系统状态
#[derive(Debug)]
pub struct MultiBodyModel {
    /// 系统中的所有刚体
    pub bodies: Vec<RigidBody>,
    /// 系统中的所有铰链关节
    pub joints: Vec<HingeJoint>,
    /// 自由度数量（= 关节数量）
    pub nq: usize,
}

/// 仿真状态
///
/// 使用广义坐标表示系统的完整动力学状态
///
/// ## 广义坐标说明
/// - `q`: 广义位置（关节角度）
/// - `qvel`: 广义速度（关节角速度）
/// - `qacc`: 广义加速度（关节角加速度）
/// - `qfrc`: 广义力（关节力矩）
#[derive(Debug, Clone)]
pub struct SimulationState {
    /// 广义位置（关节角度，单位：弧度）
    pub q: Vec<f32>,
    /// 广义速度（关节角速度，单位：弧度/秒）
    pub qvel: Vec<f32>,
    /// 广义加速度（关节角加速度，单位：弧度/秒²）
    pub qacc: Vec<f32>,
    /// 广义力（关节力矩，单位：牛顿·米）
    pub qfrc: Vec<f32>,
}

impl SimulationState {
    /// 创建新的仿真状态
    ///
    /// 所有状态变量初始化为零
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
    /// 创建空的多体模型
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            joints: Vec::new(),
            nq: 0,
        }
    }

    /// 添加刚体到模型
    ///
    /// # 返回
    /// 刚体的索引
    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.push(body);
        self.bodies.len() - 1
    }

    /// 添加铰链关节到模型
    ///
    /// 每个关节增加一个自由度
    pub fn add_hinge_joint(&mut self, joint: HingeJoint) {
        self.joints.push(joint);
        self.nq += 1;
    }
}

impl Default for MultiBodyModel {
    fn default() -> Self {
        Self::new()
    }
}
