//! # Hinge Chain Simulation
//!
//! 多体动力学仿真演示程序
//!
//! 本程序展示了如何使用自定义的多体动力学框架来模拟由Hinge关节连接的
//! Capsule链条在重力作用下的摆动。
//!
//! ## 模块组织
//!
//! - `multibody`: 通用的多体动力学求解框架
//! - `demos`: 示例应用（包括hinge_chain）
//! - `main`: Bevy渲染和主循环

mod demos;
mod multibody;

use bevy::prelude::*;
use demos::{create_hinge_chain, CAPSULE_HALF_LENGTH, NUM_CAPSULES};
use multibody::{rk4_step, MultiBodyModel, SimulationState};

// 仿真参数
const TIMESTEP: f32 = 0.002; // 2ms时间步长
const SUBSTEPS: usize = 1; // 每帧1个物理子步（60fps → 500Hz物理更新）

/// 物理模型资源
#[derive(Resource)]
struct PhysicsModel {
    model: MultiBodyModel,
    state: SimulationState,
    time: f32,
}

/// Capsule可视化组件
#[derive(Component)]
struct CapsuleVisual {
    body_index: usize,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Hinge Chain - Multibody Dynamics Simulation".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.1, 0.1, 0.15)))
        .add_systems(Startup, setup)
        .add_systems(Update, (physics_step, update_visuals, camera_controller))
        .run();
}

/// 初始化场景
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 创建HingeChain多体模型
    let (model, state) = create_hinge_chain();

    // 保存到资源
    commands.insert_resource(PhysicsModel {
        model,
        state,
        time: 0.0,
    });

    // 创建Capsule可视化
    let capsule_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.5, 0.3),
        metallic: 0.3,
        perceptual_roughness: 0.5,
        ..default()
    });

    let capsule_mesh = meshes.add(Capsule3d::new(
        demos::hinge_chain::CAPSULE_RADIUS,
        2.0 * CAPSULE_HALF_LENGTH,
    ));

    for i in 0..NUM_CAPSULES {
        // Bevy的Capsule3d默认沿Y轴，我们需要旋转90度使其沿Z轴
        // 与MuJoCo的坐标系一致（capsule沿Z轴）
        let rotation = Quat::from_rotation_x(std::f32::consts::PI / 2.0);
        commands.spawn((
            Mesh3d(capsule_mesh.clone()),
            MeshMaterial3d(capsule_material.clone()),
            Transform {
                translation: Vec3::new(0.0, 0.0, -(i as f32) * 2.0 * CAPSULE_HALF_LENGTH),
                rotation,
                scale: Vec3::ONE,
            },
            CapsuleVisual { body_index: i },
        ));
    }

    // 创建锚点可视化（红色小球）
    let anchor_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.0, 0.0),
        ..default()
    });
    let sphere_mesh = meshes.add(Sphere::new(0.08));

    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(anchor_material),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // 添加光照
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(3.0, 5.0, 3.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-3.0, 5.0, -3.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));

    // 添加相机（更新为Z轴向上的坐标系）
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 3.0, 0.0).looking_at(Vec3::new(0.0, 0.0, -1.5), Dir3::Z),
    ));

    println!("\n=== Hinge Chain Simulation Started ===");
    println!("Control:");
    println!("  Arrow Keys: Rotate camera");
    println!("  Up/Down: Zoom in/out");
    println!("=====================================\n");
}

/// 物理仿真步进
fn physics_step(mut physics: ResMut<PhysicsModel>) {
    // 多个子步提高数值稳定性
    for _ in 0..SUBSTEPS {
        let PhysicsModel {
            ref mut model,
            ref mut state,
            ..
        } = *physics;
        rk4_step(model, state, TIMESTEP);
        physics.time += TIMESTEP;
    }

    // 定期打印调试信息
    if (physics.time / TIMESTEP) as i32 % 500 == 0 {
        println!(
            "Time: {:.2}s | q[0]: {:.3} rad ({:.1}°) | qvel[0]: {:.3} rad/s",
            physics.time,
            physics.state.q[0],
            physics.state.q[0].to_degrees(),
            physics.state.qvel[0]
        );
    }
}

/// 更新可视化
fn update_visuals(physics: Res<PhysicsModel>, mut query: Query<(&mut Transform, &CapsuleVisual)>) {
    for (mut transform, visual) in query.iter_mut() {
        let body = &physics.model.bodies[visual.body_index];

        // 从物理引擎同步位置和姿态
        transform.translation = body.position;

        // Bevy的Capsule3d默认沿Y轴，需要旋转使其沿Z轴
        // 先应用物理引擎的姿态，再应用基础旋转
        let base_rotation = Quat::from_rotation_x(std::f32::consts::PI / 2.0);
        transform.rotation = body.orientation * base_rotation;
    }
}

/// 相机控制器
fn camera_controller(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    for mut transform in query.iter_mut() {
        let mut rotation_delta: f32 = 0.0;
        let mut zoom_delta: f32 = 0.0;

        if keyboard.pressed(KeyCode::ArrowLeft) {
            rotation_delta -= 1.0;
        }
        if keyboard.pressed(KeyCode::ArrowRight) {
            rotation_delta += 1.0;
        }
        if keyboard.pressed(KeyCode::ArrowUp) {
            zoom_delta -= 1.0;
        }
        if keyboard.pressed(KeyCode::ArrowDown) {
            zoom_delta += 1.0;
        }

        // 绕Z轴旋转（Z轴向上的坐标系）
        if rotation_delta.abs() > 0.01 {
            let rotation = Quat::from_rotation_z(rotation_delta * time.delta_secs());
            let new_pos = rotation * transform.translation;
            transform.translation = new_pos;
            transform.look_at(Vec3::new(0.0, 0.0, -1.5), Dir3::Z);
        }

        // 缩放
        if zoom_delta.abs() > 0.01 {
            let direction = transform.translation.normalize();
            transform.translation += direction * zoom_delta * time.delta_secs() * 2.0;
        }
    }
}
