//! # 几何工具模块
//!
//! 提供计算刚体质量和惯性张量的工具函数

use bevy::math::{Mat3, Vec3};
use std::f32::consts::PI;

/// 计算Capsule的质量和惯性张量
///
/// Capsule由一个圆柱体和两个半球（组成一个完整球体）构成。
///
/// ## 参数
/// - `length`: 圆柱体长度（两个半球中心之间的距离）
/// - `radius`: Capsule半径
/// - `density`: 材料密度 (kg/m³)
///
/// ## 返回值
/// - `(mass, inertia)`: 质量(kg)和惯性张量(kg·m²)
///
/// ## 数学推导
///
/// **体积**:
/// ```text
/// V_cylinder = π * r² * h
/// V_sphere = (4/3) * π * r³
/// V_total = V_cylinder + V_sphere
/// ```
///
/// **质量**:
/// ```text
/// m = ρ * V_total
/// ```
///
/// **惯性张量** (假设capsule沿z轴对齐):
///
/// 圆柱体惯性（相对质心）:
/// ```text
/// I_cyl_xx = I_cyl_yy = m_cyl/12 * (3r² + h²)
/// I_cyl_zz = m_cyl/2 * r²
/// ```
///
/// 球体惯性:
/// ```text
/// I_sphere = (2/5) * m_sphere * r²
/// ```
///
/// 使用平行轴定理组合两个半球（位于±h/2处）:
/// ```text
/// I_hemisphere = I_cm + m * d²
/// ```
///
/// 其中 d = h/2 是半球质心到capsule质心的距离
///
pub fn capsule_inertia(length: f32, radius: f32, density: f32) -> (f32, Mat3) {
    // 体积计算
    let cylinder_volume = PI * radius * radius * length;
    let sphere_volume = (4.0 / 3.0) * PI * radius * radius * radius;
    let total_volume = cylinder_volume + sphere_volume;
    let mass = density * total_volume;

    // 质量分量
    let cylinder_mass = density * cylinder_volume;
    let sphere_mass = density * sphere_volume;

    // 圆柱体惯性（沿z轴）
    // Ixx = Iyy = m/12 * (3*r² + h²)
    // Izz = m/2 * r²
    let cyl_ixx = cylinder_mass / 12.0 * (3.0 * radius * radius + length * length);
    let cyl_izz = cylinder_mass / 2.0 * radius * radius;

    // 球体在原点的惯性
    let sphere_inertia = 0.4 * sphere_mass * radius * radius;

    // 平行轴定理：两个半球位于±length/2处
    let hemisphere_mass = sphere_mass / 2.0;
    let d = length / 2.0;
    let hemisphere_ixx = sphere_inertia / 2.0 + hemisphere_mass * d * d;

    // 总惯性（两个半球）
    let total_ixx = cyl_ixx + 2.0 * hemisphere_ixx;
    let total_izz = cyl_izz + sphere_inertia;

    // 惯性张量（capsule沿Z轴对齐，与MuJoCo一致）
    // Ixx, Iyy 是垂直于长轴的惯性
    // Izz 是沿长轴的惯性
    let inertia = Mat3::from_diagonal(Vec3::new(total_ixx, total_ixx, total_izz));

    (mass, inertia)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_capsule_inertia() {
        // 测试：长度0.5m，半径0.05m，密度1000kg/m³的capsule
        let (mass, inertia) = capsule_inertia(0.5, 0.05, 1000.0);

        assert!(mass > 0.0, "质量应该为正");
        assert!(inertia.x_axis.x > 0.0, "惯性应该为正");

        println!("Capsule 质量: {:.4} kg", mass);
        println!("Capsule 惯性: {:?}", inertia);

        // 预期质量约为 4-5 kg（圆柱体+球体）
        assert!((mass - 4.3).abs() < 1.0, "质量应该在合理范围内");
    }
}
