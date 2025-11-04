# Hinge Chain 物理总览

本文档梳理 `hinge_chain` Rust 项目的全部物理建模与求解流程，帮助在阅读代码时快速定位物理含义与实现细节。

## 1. 模拟范围与总体流程

- 引擎基于广义坐标系的树状多体系统，只包含刚体与单自由度铰链关节（`hinge_chain/src/multibody/mod.rs:1`）。
- 物理步进遵循 MuJoCo 的前向动力学管线：前向运动学 → 速度/惯性量 → 广义力 → 质量矩阵 → 加速度 → 时间积分（`hinge_chain/src/multibody/mod.rs:12`）。
- 重力固定为世界坐标系的 -Z 方向 `g = (0, 0, -9.81)` m/s²（`hinge_chain/src/multibody/mod.rs:74`）。

## 2. 坐标系与符号约定

- 世界坐标系：右手系，Z 轴向上；渲染与 MuJoCo 配置保持一致（`hinge_chain/src/demos/hinge_chain.rs:52`）。
- Body 坐标系：与刚体一同移动，质心位于局部原点，长轴沿 Z 轴（胶囊几何假设）。
- Joint 坐标系：以子体局部坐标的关节锚点为原点，轴方向在父体坐标内定义，并在计算中转换到世界系（`hinge_chain/src/multibody/kinematics.rs:91`）。
- 空间向量格式采用 Featherstone/MuJoCo 约定 `[ω_x, ω_y, ω_z, v_x, v_y, v_z]`（`hinge_chain/src/multibody/spatial_algebra.rs:7`）。

## 3. 物理基本单元

### 3.1 空间惯性与刚体

- `SpatialInertia` 保存质量、3×3 旋转惯性与质心偏移（`hinge_chain/src/multibody/model.rs:19`），支持复合惯性相加以供 CRBA 使用（`hinge_chain/src/multibody/model.rs:41`）。
- `RigidBody` 存储质量属性、当前姿态/速度与 6D 空间速度缓冲区（`hinge_chain/src/multibody/model.rs:73`）。所有物体初始化于原点并在前向运动学后更新。

### 3.2 铰链关节

- `HingeJoint` 描述父子关系、关节轴、几何偏移、阻尼与附加转子惯量（`hinge_chain/src/multibody/model.rs:125`）。
- 运行时缓存运动子空间 `cdof` 与其时间导数 `cdof_dot`，由速度阶段计算得到（`hinge_chain/src/multibody/velocity.rs:37`）。

### 3.3 系统与状态向量

- `MultiBodyModel` 管理刚体/关节集合与自由度数量 `nq`（`hinge_chain/src/multibody/model.rs:168`）。
- `SimulationState` 保存广义位置 `q`、速度 `qvel`、加速度 `qacc` 与广义力 `qfrc` 数组，全部以关节序排列（`hinge_chain/src/multibody/model.rs:195`）。

## 4. 几何与惯性建模

- 胶囊几何质量与惯量由 `capsule_inertia(length, radius, density)` 计算（`hinge_chain/src/multibody/geometry.rs:8`）。实现按圆柱体 + 球体组合推导体积与惯性，并在 Z 轴对齐下生成对角惯性张量（`hinge_chain/src/multibody/geometry.rs:65`）。
- 质心假设位于几何中心，使 `com_offset ≈ 0`，从而在动力学中可忽略平移惯量耦合（`hinge_chain/src/multibody/model.rs:24`）。

## 5. 前向动力学管线

### 5.1 前向运动学

- `forward_kinematics` 依据广义位置 `q` 递归更新每个刚体的世界姿态与质心位置（`hinge_chain/src/multibody/kinematics.rs:53`）。
  - 关节旋转通过轴角插值并保持锚点固定，确保偏心旋转后 anchor 不漂移（`hinge_chain/src/multibody/kinematics.rs:106`）。
  - 同步更新线速度与角速度的父子继承部分，为后续阶段提供初值（`hinge_chain/src/multibody/kinematics.rs:132`）。

### 5.2 子树质心

- `compute_subtree_com` 以后向累积方式计算每个 body 的子树质心位置，用于确定运动自由度的线速度分量（`hinge_chain/src/multibody/subtree_com.rs:28`）。

### 5.3 空间速度与运动子空间

- `compute_velocities` 在树上前向传播，生成：
  - 空间速度 `cvel = cvel_parent + cdof * qvel`，反映当前关节速度贡献（`hinge_chain/src/multibody/velocity.rs:29`）。
  - 运动子空间 `cdof = [axis_world, axis_world × r_COM]`，其中 `r_COM` 取自锚点到子树质心的向量（`hinge_chain/src/multibody/velocity.rs:67`）。
  - Lie bracket `cdof_dot = cvel_parent × cdof`，提供科里奥利项所需的加速度贡献（`hinge_chain/src/multibody/velocity.rs:81`）。

### 5.4 广义力（RNE）

- `compute_generalized_forces` 实现完整的递归牛顿-欧拉算法（`hinge_chain/src/multibody/dynamics.rs:61`）。
  - 前向递归汇总惯性加速度与 `cdof_dot * qvel`，加上重力初值 `+g` 以直接得到 `τ = M*qacc` 形式（`hinge_chain/src/multibody/dynamics.rs:84`）。
  - 空间力由 `I * cacc + cvel × (I * cvel)` 求得，第二项对应科里奥利/离心力（`hinge_chain/src/multibody/dynamics.rs:138`）。
  - 后向累积将子体力矩传回父体，再沿 `cdof` 投影到关节广义力，并减去线性阻尼 `-damping * qvel`（`hinge_chain/src/multibody/dynamics.rs:169`）。

### 5.5 质量矩阵（CRBA）

- `compute_mass_matrix` 采用 Composite Rigid Body Algorithm：
  1. 初始化每个 body 的空间惯性（`hinge_chain/src/multibody/dynamics.rs:230`）。
  2. 由叶到根累加子树惯性，获得复合刚体惯性 `crb`（`hinge_chain/src/multibody/dynamics.rs:239`）。
  3. 将关节轴投影到世界系并与复合惯性双重投影，生成矩阵元素 `M[i,j]`，同时加上电机惯量（`hinge_chain/src/multibody/dynamics.rs:248`）。

### 5.6 线性系统求解

- `compute_acceleration` 构建完整质量矩阵后，使用部分选主元高斯消元求解 `M * qacc = qfrc`（`hinge_chain/src/multibody/dynamics.rs:362`、`hinge_chain/src/multibody/dynamics.rs:434`）。
- 结果写回 `state.qacc` 以供积分阶段使用。

## 6. 力学建模要点

- **重力**：通过前向递归初始加速度注入，避免额外的 `qfrc_bias` 符号转换（`hinge_chain/src/multibody/dynamics.rs:84`）。
- **科里奥利/离心力**：由 `cross_force(cvel, I*cvel)` 计算并自动并入广义力（`hinge_chain/src/multibody/dynamics.rs:146`）。
- **阻尼**：线性黏性模型 `τ_damp = -damping * qvel`，每个关节独立配置（`hinge_chain/src/multibody/dynamics.rs:179`）。
- **电机转子惯量**：以 `armature` 的形式直接加到质量矩阵对角线，代表执行器惯性（`hinge_chain/src/multibody/dynamics.rs:301`）。
- **空间代数算子**：交叉积与惯性-速度乘法遵循 Featherstone 公式，确保角线分量耦合正确（`hinge_chain/src/multibody/spatial_algebra.rs:122`、`hinge_chain/src/multibody/spatial_algebra.rs:189`）。

## 7. 时间积分策略

- `rk4_step` 封装四阶 Runge-Kutta，四次调用 `forward_dynamics` 评估导数并加权组合（`hinge_chain/src/multibody/integrator.rs:88`）。
- `forward_dynamics` 按 5 个阶段完成一次前向动力学求解，并缓存 `qfrc` 与 `qacc`（`hinge_chain/src/multibody/integrator.rs:24`）。
- 提供半隐式欧拉作为快速近似选项，适合调试但默认未启用（`hinge_chain/src/multibody/integrator.rs:166`）。

## 8. Hinge Chain 演示场景设定

- 系统包含 6 个同质胶囊，长度 0.5 m、半径 0.05 m、密度 1000 kg/m³（`hinge_chain/src/demos/hinge_chain.rs:22`）。
- 关节交替绕 X/Y 轴旋转，父子间沿 Z 轴串联，每个关节安装阻尼 0.1 N·m·s/rad 与 0.01 kg·m² 转子惯量（`hinge_chain/src/demos/hinge_chain.rs:86`、`hinge_chain/src/demos/hinge_chain.rs:117`）。
- 初始状态为第一个关节 0.9 rad 的扰动，其余角度默认 0，使系统在重力下自然摆动（`hinge_chain/src/demos/hinge_chain.rs:130`）。

## 9. 仿真循环与可视化

- 主程序以 2 ms 时间步运行 RK4 积分，Bevy 每帧调用一次物理步进（`hinge_chain/src/main.rs:21`、`hinge_chain/src/main.rs:145`）。
- 渲染将物理计算得到的姿态直接映射到 Capsule Mesh，并保持 MuJoCo 坐标约定（`hinge_chain/src/main.rs:170`）。
- 控制台定期输出首个关节的角度与角速度监控系统能量变化（`hinge_chain/src/main.rs:158`）。

## 10. 模型假设与当前限制

- 仅支持树状拓扑（无闭环或多父节点），与结构体设计一致（`hinge_chain/src/multibody/model.rs:172`）。
- 所有关节为单自由度旋转；无滑动或球铰链实现。
- 忽略碰撞与接触力，未实现外部力/控制执行器，仅考虑重力、惯性和阻尼。
- 胶囊质心与几何中心重合，未处理非均匀密度或偏心误差。
- 质量矩阵每步重算并直接消元，规模较大时需优化（代码注释中建议使用 LDLᵀ 分解，`hinge_chain/src/multibody/integrator.rs:83`）。

## 11. 单元测试与验证

- 几何、空间代数、运动学、动力学与积分模块均附带基础测试，验证典型物理性质（例如 `hinge_chain/src/multibody/geometry.rs:95`、`hinge_chain/src/multibody/dynamics.rs:455`）。
- 演示工厂函数检查链式拓扑与状态长度的一致性，防止配置错误（`hinge_chain/src/demos/hinge_chain.rs:150`）。

通过以上结构，可以将项目中的每个物理量、算法与对应代码快速对应起来，便于继续深入实现或进行数值验证。
