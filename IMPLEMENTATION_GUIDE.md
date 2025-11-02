# MuJoCo 动力学算法 Rust 实现指南

## 概述

本指南总结了 MuJoCo 中的四个关键动力学算法的实现细节，帮助你在 Rust 中准确复现这些算法。

**文档位置**:
- `MUJOCO_DYNAMICS_ANALYSIS.md` - 高层算法和数据结构分析
- `MUJOCO_CODE_DETAILS.md` - 代码级实现细节和伪代码

**源代码位置**:
```
mujoco/src/engine/
├── engine_forward.c           # 前向动力学、RK4
├── engine_inverse.c           # 逆向动力学
├── engine_core_smooth.c       # CRBA、RNE、因式分解
└── engine_core_smooth.h       # 公共接口声明
```

---

## 四个关键算法速查表

### 1. RK4 积分器 (Runge-Kutta 4阶)

| 属性 | 值 |
|------|-----|
| 位置 | engine_forward.c, 行 890-992 |
| 函数 | `mj_RungeKutta(m, d, N=4)` |
| 目的 | 时间积分，更新状态 |
| 输入 | qpos, qvel, act, qacc, act_dot |
| 输出 | 更新后的 qpos, qvel, act |

**关键系数**:
```c
RK4_A = [0.5, 0, 0; 0, 0.5, 0; 0, 0, 1]  // 3个中间点系数
RK4_B = [1/6, 1/3, 1/3, 1/6]              // 最终权重
```

**算法特点**:
- 半隐式积分
- 四个评估点（初始 + 3个中间）
- 四元数保持规范化
- 4阶精度

**Rust 实现清单**:
- [ ] 预计算时间点 T[i] = t + C[i]*h
- [ ] 初始化 X[0], F[0]（已有加速度）
- [ ] 循环 3 次中间点，每次调用完整前向动力学
- [ ] 加权汇总得到最终增量
- [ ] 应用到状态，规范化四元数

---

### 2. 前向动力学 (Forward Dynamics)

| 属性 | 值 |
|------|-----|
| 位置 | engine_forward.c, 行 1101-1150 |
| 函数 | `mj_forward(m, d)` |
| 目的 | 计算加速度及约束力 |
| 输入 | qpos, qvel, ctrl, act |
| 输出 | qacc, qfrc_constraint, efc_force |

**执行流程**:
1. **mj_fwdPosition()** - 运动学、质量矩阵、因式分解
2. **mj_fwdVelocity()** - 速度导出、RNE(flg_acc=0) → qfrc_bias
3. **mj_fwdActuation()** - 执行器力计算
4. **mj_fwdAcceleration()** - 汇总力、求解 M*a = f
5. **mj_fwdConstraint()** - 约束求解

**关键方程**:
```
qfrc_bias = mj_rne(q, v, flg_acc=0)  // 科里奥利/离心力
qfrc_smooth = qfrc_actuator + qfrc_applied - qfrc_bias - qfrc_passive + xfrc
qacc_smooth = M^(-1) * qfrc_smooth
```

**Rust 实现清单**:
- [ ] 正向运动学 (mj_kinematics)
- [ ] CoM 相关计算 (mj_comPos) - cinert, cdof
- [ ] CRBA 质量矩阵 (mj_crb)
- [ ] 质量矩阵因式分解 (mj_factorM)
- [ ] RNE with flg_acc=0 (mj_rne)
- [ ] 力汇总
- [ ] 线性系统求解 (mj_solveM)
- [ ] 约束求解（可选首次）

---

### 3. CRBA（质量矩阵）

| 属性 | 值 |
|------|-----|
| 位置 | engine_core_smooth.c, 行 1514-1569 |
| 函数 | `mj_crb(m, d)` 和 `mj_makeM(m, d)` |
| 目的 | 计算质量矩阵 M(q) |
| 输入 | cinert[], cdof[], dof_armature[] |
| 输出 | M[] (稀疏) |

**算法步骤**:
1. 初始化复合惯性：`crb = cinert`
2. 向后递推：累积子树惯性到父体
3. 前向递推：对每个 dof i，计算 `M[i,j] = cdof[j]^T * crb[i] * cdof[i]`
4. 结果是对称正定矩阵（SPD）

**关键计算**:
```rust
buf = crb[body_i] * cdof[i]  // 惯性-速度投影
M[i,j] = cdof[j] · buf        // 点积
```

**Rust 实现清单**:
- [ ] cinert 初始化（在 mj_comPos 中）
- [ ] cdof 初始化（在 mj_comPos 中）
- [ ] 复合惯性后向递推
- [ ] 稀疏矩阵存储和寻址
- [ ] 惯性向量乘法 (mju_mulInertVec)
- [ ] 点积计算

**数据结构**:
```rust
// 稀疏矩阵 M（压缩行存储）
M_rowadr[i]       // 行 i 的起始地址
M_rownnz[i]       // 行 i 的非零元素数
M_colind[adr]     // 列索引
M[start:start+nnz] // 该行的数值
```

---

### 4. RNE（递归牛顿-欧拉）

| 属性 | 值 |
|------|-----|
| 位置 | engine_core_smooth.c, 行 2068-2119 |
| 函数 | `mj_rne(m, d, flg_acc, result[])` |
| 目的 | 计算广义力 τ = M*a + C(q,v) 或仅 C |
| 输入 | qpos, qvel, qacc (如果 flg_acc=1) |
| 输出 | result[] (广义力) |

**关键参数**:
- `flg_acc = 1`: 包含惯性项，τ = M*a + C
- `flg_acc = 0`: 仅偏差项，τ = C （用于 qfrc_bias）

**算法三阶段**:
1. **前向递推**（身体1到nbody-1）:
   - 计算身体加速度：cacc[i] = cacc[parent] + cdof_dot*v + cdof*a
   - 计算身体力：cfrc[i] = I*cacc + ω × (I*ω)

2. **后向递推**（nbody-1到1）:
   - 累积子树力：cfrc[parent] += cfrc[i]

3. **投影**:
   - 广义力：result[i] = cdof[i] · cfrc[body_i]

**Rust 实现清单**:
- [ ] 初始化世界加速度（-gravity）
- [ ] 前向身体加速度计算
- [ ] 身体力计算（惯性 + 离心）
- [ ] 后向力累积
- [ ] 投影到关节空间

**关键向量运算**:
```rust
// 6维空间向量: [v_lin_x, v_lin_y, v_lin_z, v_ang_x, v_ang_y, v_ang_z]
// 惯性向量乘法：I * v
// 叉积：w × f
// 点积：u · v
```

---

## 数据结构映射

### mjData 核心数组

| C 名称 | 大小 | Rust 对应 | 含义 |
|--------|------|----------|------|
| qpos[] | nq | state.qpos | 广义坐标 |
| qvel[] | nv | state.qvel | 广义速度 |
| qacc[] | nv | state.qacc | 广义加速度 |
| M[] | nC | inertia.M | 质量矩阵（稀疏） |
| cinert[] | 10*nbody | body_cinert | CoM处复合惯性 |
| cdof[] | 6*nv | dof_cdof | 运动自由度向量 |
| cvel[] | 6*nbody | body_cvel | 身体速度 |
| cacc[] | 6*nbody | body_cacc | 身体加速度 |
| qfrc_bias[] | nv | forces.qfrc_bias | 科里奥利/离心力 |
| qfrc_actuator[] | nv | forces.qfrc_actuator | 执行器力 |
| qfrc_smooth[] | nv | forces.qfrc_smooth | 汇总力 |

### mjModel 索引数据

| C 名称 | 大小 | 含义 |
|--------|------|------|
| M_rowadr[] | nv | 质量矩阵行地址 |
| M_colind[] | nC | 列索引 |
| M_rownnz[] | nv | 每行非零元素数 |
| body_parentid[] | nbody | 父身体ID |
| dof_parentid[] | nv | 父DOF ID |
| body_dofadr[] | nbody | 身体第一个DOF地址 |
| body_dofnum[] | nbody | 身体DOF数 |

---

## 实现顺序建议

### Phase 1: 基础设施 (Week 1)
1. 数据结构定义（State, Model, Inertia）
2. 向量/矩阵操作库（3x3, 6x6, 点积、叉积）
3. 四元数操作（乘法、规范化、轴角变换）
4. 稀疏矩阵存储和索引

### Phase 2: 运动学和质量 (Week 2)
1. ✓ mj_kinematics - 正向运动学
2. ✓ mj_comPos - CoM 相关计算
3. ✓ mj_crb/mj_makeM - CRBA 质量矩阵
4. ✓ mj_factorM - 质量矩阵因式分解
5. ✓ mj_solveM - 线性系统求解

### Phase 3: 力和加速度 (Week 3)
1. ✓ mj_rne - RNE 算法
2. ✓ mj_fwdVelocity - 速度导出（含 qfrc_bias）
3. ✓ mj_fwdActuation - 执行器力
4. ✓ mj_fwdAcceleration - 汇总和求解
5. ✓ 初步测试：无约束系统

### Phase 4: 积分和完整系统 (Week 4)
1. ✓ mj_integratePos - 位置积分
2. ✓ mj_Euler - Euler 积分器
3. ✓ mj_RungeKutta - RK4 积分器
4. ✓ 完整 mj_forward 流程
5. ✓ 测试和验证

---

## 关键实现细节

### 1. 稀疏矩阵寻址

```rust
fn get_M_element(M: &[f64], rowadr: &[usize], rownnz: &[usize], 
                 i: usize, j: usize) -> f64 {
    let start = rowadr[i];
    let nnz = rownnz[i];
    // M[i,j] 在 M[start..start+nnz] 中
    // 列索引由 colind[] 给出
    // 对于树形链，列是递增的
}
```

### 2. 惯性向量乘法

```rust
// cinert 是 10 元向量：[mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, pad, pad, pad]
// cdof 是 6 元向量：[v_lin, v_ang] = [vx, vy, vz, wx, wy, wz]
fn inertia_vector_multiply(I: &[f64; 10], v: &[f64; 6]) -> [f64; 6] {
    let mass = I[0];
    let inertia = [[I[1], I[4], I[5]],
                   [I[4], I[2], I[6]],
                   [I[5], I[6], I[3]]];
    
    let v_lin = &v[0..3];
    let v_ang = &v[3..6];
    
    let I_v_ang = inertia * v_ang;  // 3x3 * 3
    let m_v_lin = mass * v_lin;
    
    [
        m_v_lin[0] + cross(I_v_ang, v_lin)[0],
        m_v_lin[1] + cross(I_v_ang, v_lin)[1],
        m_v_lin[2] + cross(I_v_ang, v_lin)[2],
        (inertia * v_ang)[0] + cross(v_lin, m_v_lin)[0],
        (inertia * v_ang)[1] + cross(v_lin, m_v_lin)[1],
        (inertia * v_ang)[2] + cross(v_lin, m_v_lin)[2],
    ]
}
```

### 3. CRBA 核心步骤

```rust
fn compute_mass_matrix(m: &Model, data: &mut Data) {
    // Step 1: 初始化
    for i in 0..m.nbody {
        data.crb[i].copy_from_slice(&data.cinert[10*i..10*i+10]);
    }
    
    // Step 2: 向后递推（累积子树）
    for i in (1..m.nbody).rev() {
        let parent = m.body_parentid[i];
        if parent > 0 {
            for k in 0..10 {
                data.crb[10*parent + k] += data.crb[10*i + k];
            }
        }
    }
    
    // Step 3: 清零质量矩阵
    data.M.fill(0.0);
    
    // Step 4: 前向递推（计算 M）
    for i in 0..m.nv {
        let bodyid = m.dof_bodyid[i];
        let adr = m.M_rowadr[i];
        let nnz = m.M_rownnz[i];
        let diag_idx = nnz - 1;
        
        // 加入电枢惯性
        data.M[adr + diag_idx] = m.dof_armature[i];
        
        // 计算 buf = crb[body_i] * cdof[i]
        let buf = inertia_vector_multiply(&data.cinert[10*bodyid..], &data.cdof[6*i..]);
        
        // 对所有祖先 dof 计算 M[i,j]
        let mut j = i;
        let mut M_addr = adr + diag_idx;
        while j >= 0 {
            let dot = dot_product(&data.cdof[6*j..], &buf);
            data.M[M_addr] += dot;
            
            j = m.dof_parentid[j];
            M_addr += 1;
        }
    }
}
```

### 4. RNE 两遍算法

```rust
fn compute_generalized_forces(m: &Model, data: &mut Data, 
                              flg_acc: bool, result: &mut [f64]) {
    // 初始化（世界加速度 = -重力）
    let mut cacc = vec![[0.0; 6]; m.nbody];
    cacc[0][3] = -m.gravity[0]; // 重力在角加速度分量
    cacc[0][4] = -m.gravity[1];
    cacc[0][5] = -m.gravity[2];
    
    // 前向递推：计算加速度和力
    let mut cfrc = vec![[0.0; 6]; m.nbody];
    for i in 1..m.nbody {
        let parent = m.body_parentid[i];
        let bda = m.body_dofadr[i];
        let ndof = m.body_dofnum[i];
        
        // 累积加速度
        cacc[i] = cacc[parent];
        
        // 加入 cdof_dot * qvel（科里奥利项）
        for k in 0..ndof {
            let dof = bda + k;
            for d in 0..6 {
                cacc[i][d] += data.cdof_dot[6*dof + d] * data.qvel[dof];
            }
        }
        
        // 加入 cdof * qacc（加速度项）
        if flg_acc {
            for k in 0..ndof {
                let dof = bda + k;
                for d in 0..6 {
                    cacc[i][d] += data.cdof[6*dof + d] * data.qacc[dof];
                }
            }
        }
        
        // 计算身体力
        cfrc[i] = inertia_vector_multiply(&data.cinert[10*i..], &cacc[i]);
        
        // 加入离心项：ω × (I*ω)
        let I_omega = inertia_vector_multiply(&data.cinert[10*i..], &data.cvel[6*i..]);
        let centrifugal = cross_force(&data.cvel[6*i..], &I_omega);
        for d in 0..6 {
            cfrc[i][d] += centrifugal[d];
        }
    }
    
    // 后向递推：累积子树力
    for i in (1..m.nbody).rev() {
        let parent = m.body_parentid[i];
        if parent > 0 {
            for d in 0..6 {
                cfrc[parent][d] += cfrc[i][d];
            }
        }
    }
    
    // 投影到关节空间
    for i in 0..m.nv {
        let bodyid = m.dof_bodyid[i];
        result[i] = dot_product(&data.cdof[6*i..], &cfrc[bodyid]);
    }
}
```

---

## 验证和测试

### 单元测试清单

```rust
#[test]
fn test_kinematics_simple_hinge() {
    // 单个铰链链，检查位置/方向正确性
}

#[test]
fn test_crba_mass_matrix_symmetry() {
    // 验证 M 是对称的
}

#[test]
fn test_crba_mass_matrix_positive_definite() {
    // 验证 M 特征值都是正的
}

#[test]
fn test_rne_gravity_torque() {
    // 单个身体，验证重力产生的扭矩
}

#[test]
fn test_forward_dynamics_free_fall() {
    // 自由下落，应该产生向下的加速度
}

#[test]
fn test_rk4_energy_conservation() {
    // 无摩擦系统能量应近似守恒
}

#[test]
fn test_consistency_with_mujoco() {
    // 与 MuJoCo 数值对比（如果可行）
}
```

### 对标值来源

```
MuJoCo 源代码:
- qpos, qvel → 运动学 → xpos, xmat
- xpos, xmat, xquat → CRBA → M
- qvel, qacc → RNE → qfrc_bias
```

---

## 常见陷阱

### 1. 四元数规范化
- 每次正向运动学后必须规范化四元数
- 积分时也要规范化

### 2. 稀疏矩阵索引
- 列索引存储在 M_colind[]
- 行 i 的非零元在 M[rowadr[i]:rowadr[i]+rownnz[i]]
- **不能随意访问**

### 3. 空间向量运算
- 速度/加速度是 6 元向量（线速度 + 角速度）
- 力/力矩也是 6 元向量
- 叉积公式不同于 3D 向量

### 4. 离心/科里奥利项
- qfrc_bias 包含的是 **C 项**（科里奥利/离心力）
- 在力平衡中应该**相减**：F_external - C = M*a
- RNE 计算的顺序很关键

### 5. 质量矩阵分解
- L*D*L^T 分解用于快速求解
- 对角线元素容易变得太小，需要最小值检查
- 回代需要按正确的顺序进行

---

## 参考资源

### MuJoCo 源文件
- `engine_forward.c` - 前向动力学、积分器
- `engine_core_smooth.c` - CRBA、RNE
- `engine_util_spatial.c` - 空间向量运算
- `engine_util_blas.c` - 矩阵/向量操作

### 理论参考
- Featherstone, R. (2008). Rigid Body Dynamics Algorithms. Springer.
  - 第6章：CRBA 质量矩阵
  - 第4章：正向动力学
  - 第7章：逆向动力学

- Siciliano, B. et al. (2008). Robotics: Modeling, Planning and Control.
  - 第3章：运动学
  - 第7章：动力学和控制

---

## 联系和反馈

如有疑问或发现问题，请参考：
1. MuJoCo 源代码注释
2. MuJoCo 文档和论文
3. 本指南的详细版本（MUJOCO_DYNAMICS_ANALYSIS.md）

