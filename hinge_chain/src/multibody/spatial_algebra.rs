//! # 空间代数模块
//!
//! 实现6D空间向量的运算，用于多体动力学计算。
//!
//! 基于Featherstone的空间代数理论。
//!
//! ## 空间向量格式
//!
//! 6D空间向量的布局:
//! ```text
//! v[6] = [ω_x, ω_y, ω_z, v_x, v_y, v_z]
//!        [角速度/力矩,  线速度/力]
//! ```

use bevy::math::Vec3;

/// 空间运动向量 (Spatial Motion Vector)
///
/// 表示6D运动: [角速度, 线速度]
#[derive(Debug, Clone, Copy)]
pub struct SpatialMotion {
    /// 角速度分量 (ω)
    pub angular: Vec3,
    /// 线速度分量 (v)
    pub linear: Vec3,
}

impl SpatialMotion {
    pub fn new(angular: Vec3, linear: Vec3) -> Self {
        Self { angular, linear }
    }

    pub fn zero() -> Self {
        Self {
            angular: Vec3::ZERO,
            linear: Vec3::ZERO,
        }
    }

    /// 从数组创建 [ω_x, ω_y, ω_z, v_x, v_y, v_z]
    pub fn from_array(arr: &[f32; 6]) -> Self {
        Self {
            angular: Vec3::new(arr[0], arr[1], arr[2]),
            linear: Vec3::new(arr[3], arr[4], arr[5]),
        }
    }

    /// 转换为数组
    pub fn to_array(&self) -> [f32; 6] {
        [
            self.angular.x,
            self.angular.y,
            self.angular.z,
            self.linear.x,
            self.linear.y,
            self.linear.z,
        ]
    }

    /// 标量乘法
    pub fn scale(&self, s: f32) -> Self {
        Self {
            angular: self.angular * s,
            linear: self.linear * s,
        }
    }

    /// 加法
    pub fn add(&self, other: &Self) -> Self {
        Self {
            angular: self.angular + other.angular,
            linear: self.linear + other.linear,
        }
    }
}

/// 空间力向量 (Spatial Force Vector)
///
/// 表示6D力: [力矩, 力]
#[derive(Debug, Clone, Copy)]
pub struct SpatialForce {
    /// 力矩分量 (τ)
    pub torque: Vec3,
    /// 力分量 (f)
    pub force: Vec3,
}

impl SpatialForce {
    pub fn new(torque: Vec3, force: Vec3) -> Self {
        Self { torque, force }
    }

    pub fn zero() -> Self {
        Self {
            torque: Vec3::ZERO,
            force: Vec3::ZERO,
        }
    }

    /// 加法
    pub fn add(&self, other: &Self) -> Self {
        Self {
            torque: self.torque + other.torque,
            force: self.force + other.force,
        }
    }
}

impl std::ops::AddAssign for SpatialForce {
    fn add_assign(&mut self, other: Self) {
        self.torque += other.torque;
        self.force += other.force;
    }
}

/// 空间运动叉积 (Motion Cross-Product)
///
/// 计算 res = vel × v (Lie bracket)
///
/// ## 数学公式
///
/// ```text
/// res[0:3] = -ω × ω_v
/// res[3:6] = -ω × v_v - v × ω_v
/// ```
///
/// ## 参数
/// - `vel`: 当前速度 [ω, v]
/// - `v`: 要叉乘的运动向量 [ω_v, v_v]
///
/// ## 返回
/// Lie bracket [vel, v]
///
pub fn cross_motion(vel: &SpatialMotion, v: &SpatialMotion) -> SpatialMotion {
    let omega = vel.angular;
    let v_lin = vel.linear;
    let omega_v = v.angular;
    let v_v = v.linear;

    // 角速度部分: -ω × ω_v
    let res_angular = -omega.cross(omega_v);

    // 线速度部分: -ω × v_v - v × ω_v
    let res_linear = -omega.cross(v_v) - v_lin.cross(omega_v);

    SpatialMotion::new(res_angular, res_linear)
}

/// 空间力叉积 (Force Cross-Product)
///
/// 计算 res = vel ×* f (对偶算子)
///
/// ## 数学公式
///
/// ```text
/// res[0:3] = -ω × τ - v × f
/// res[3:6] = -ω × f
/// ```
///
/// ## 用途
/// 计算离心力: cvel ×* (I * cvel)
///
#[allow(dead_code)]
pub fn cross_force(vel: &SpatialMotion, f: &SpatialForce) -> SpatialForce {
    let omega = vel.angular;
    let v = vel.linear;
    let tau = f.torque;
    let force = f.force;

    // 力矩部分: -ω × τ - v × f
    let res_torque = -omega.cross(tau) - v.cross(force);

    // 力部分: -ω × f
    let res_force = -omega.cross(force);

    SpatialForce::new(res_torque, res_force)
}

/// 空间惯性向量乘法
///
/// 计算 f = I * v，其中I是空间惯性矩阵
///
/// ## 参数
/// - `mass`: 质量
/// - `inertia`: 3x3旋转惯性张量
/// - `com_offset`: 质心偏移（对Capsule ≈ 0）
/// - `v`: 空间速度
///
/// ## 返回
/// 空间力 f = I * v
///
#[allow(dead_code)]
pub fn inertia_mul_motion(
    mass: f32,
    inertia: bevy::math::Mat3,
    com_offset: Vec3,
    v: &SpatialMotion,
) -> SpatialForce {
    let omega = v.angular;
    let vel = v.linear;

    // 力矩部分: τ = I_rot * ω + m * (com × vel)
    let torque = inertia * omega + mass * com_offset.cross(vel);

    // 力部分: f = m * vel + m * (com × ω)
    let force = mass * vel + mass * com_offset.cross(omega);

    SpatialForce::new(torque, force)
}

/// 点积投影 (Dot Product)
///
/// 计算空间运动与空间力的点积，用于投影到关节空间
///
/// ## 数学公式
///
/// ```text
/// dot = ω·τ + v·f
/// ```
///
#[allow(dead_code)]
pub fn dot_motion_force(motion: &SpatialMotion, force: &SpatialForce) -> f32 {
    motion.angular.dot(force.torque) + motion.linear.dot(force.force)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cross_motion() {
        let vel = SpatialMotion::new(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        let v = SpatialMotion::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0));

        let res = cross_motion(&vel, &v);

        // -ω × ω_v = -[1,0,0] × [0,1,0] = -[0,0,1] = [0,0,-1]
        assert!((res.angular.z + 1.0).abs() < 1e-6);
        assert!(res.angular.x.abs() < 1e-6);
        assert!(res.angular.y.abs() < 1e-6);
    }

    #[test]
    fn test_spatial_motion_ops() {
        let m1 = SpatialMotion::new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(4.0, 5.0, 6.0));
        let m2 = SpatialMotion::new(Vec3::new(0.5, 1.0, 1.5), Vec3::new(2.0, 2.5, 3.0));

        let sum = m1.add(&m2);
        assert!((sum.angular.x - 1.5).abs() < 1e-6);
        assert!((sum.linear.z - 9.0).abs() < 1e-6);

        let scaled = m1.scale(2.0);
        assert!((scaled.angular.y - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_inertia_mul_motion() {
        let mass = 1.0;
        let inertia = bevy::math::Mat3::from_diagonal(Vec3::new(0.1, 0.1, 0.1));
        let com_offset = Vec3::ZERO;
        let v = SpatialMotion::new(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));

        let f = inertia_mul_motion(mass, inertia, com_offset, &v);

        // τ = I * ω = [0.1, 0, 0] * [1, 0, 0] = [0.1, 0, 0]
        assert!((f.torque.x - 0.1).abs() < 1e-6);

        // f = m * v = 1.0 * [0, 1, 0] = [0, 1, 0]
        assert!((f.force.y - 1.0).abs() < 1e-6);
    }
}
