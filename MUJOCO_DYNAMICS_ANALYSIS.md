# MuJoCo 动力学算法实现分析

## 目录结构
```
mujoco_hinge_chain/mujoco/src/engine/
├── engine_forward.c          # 前向动力学实现
├── engine_inverse.c          # 逆向动力学实现
├── engine_core_smooth.c      # 核心平滑算法（RNE、CRBA等）
├── engine_core_smooth.h      # 核心算法声明
├── engine_core_util.c        # 工具函数
└── ...其他文件
```

## 1. RK4 积分器实现

**文件**: `/engine_forward.c` (行 890-992)

### RK4 Tableau 定义
```c
const mjtNum RK4_A[9] = {
  0.5,    0,      0,
  0,      0.5,    0,
  0,      0,      1
};

const mjtNum RK4_B[4] = {
  1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0
};
```

### 关键算法步骤

1. **初始化阶段**
   - 保存初始时间 `time = d->time`
   - 分配空间存储N个阶段的状态 X[i] 和导数 F[i]
   - 预计算 C 矩阵（阶段时间系数）和 T 矩阵（绝对时间）

2. **第一阶段 (i=0)**
   - 使用已有的状态和加速度
   - F[0] = d->qacc, X[0] = (qpos, qvel, act)

3. **中间阶段 (i=1,2,3)**
   对于每个阶段 i:
   ```
   dX = Σ(j=0 to i-1) A[i,j] * (X[j], F[j])  # 加权和
   X[i] = X[0] + dX  # 累积更新
   调用 mj_forwardSkip() 计算 F[i] = d->qacc
   ```

4. **最终更新**
   ```
   dX_final = Σ(j=0 to N-1) B[j] * (X[j], F[j])
   qpos += dX_final.position
   qvel += dX_final.velocity * h
   act += dX_final.activation * h
   ```

5. **集成特性**
   - 半隐式：位置用速度积分，速度用加速度积分
   - 通过 `mj_integratePos()` 处理四元数旋转
   - 激活状态用显式 Euler 积分

### 调用流程
```
mj_step() 
  ├─ mj_forward()      # 计算初始加速度
  └─ mj_RungeKutta(N=4)
      └─ mj_forwardSkip() × 3  # 三个中间点
```

---

## 2. 前向动力学（Forward Dynamics）

**文件**: `/engine_forward.c`

### 执行流程

```
mj_forward()
  │
  ├─ mj_fwdPosition()          # 位置相关计算
  │   ├─ mj_kinematics()       # 正向运动学
  │   ├─ mj_comPos()           # CoM 位置和质量矩阵准备
  │   ├─ mj_makeM()            # 质量矩阵计算
  │   ├─ mj_factorM()          # 质量矩阵因式分解
  │   └─ mj_collision()        # 碰撞检测
  │
  ├─ mj_fwdVelocity()          # 速度相关计算
  │   ├─ mj_comVel()           # 速度、雅可比点积
  │   ├─ mj_passive()          # 被动力（阻尼、摩擦）
  │   └─ mj_rne(flg_acc=0)    # RNE 计算偏差力 qfrc_bias
  │
  ├─ mj_fwdActuation()         # 执行器力
  │   └─ 计算 qfrc_actuator
  │
  ├─ mj_fwdAcceleration()       # 加速度计算
  │   ├─ 汇总所有非约束力
  │   ├─ qfrc_smooth = qfrc_passive - qfrc_bias + qfrc_applied + qfrc_actuator
  │   └─ qacc_smooth = M \ qfrc_smooth  # 求解线性系统
  │
  └─ mj_fwdConstraint()        # 约束处理
      ├─ mj_constraintUpdate()
      └─ 约束求解器（PGS/CG/Newton）
```

### 关键计算点

#### qfrc_bias 计算（mj_fwdVelocity）
```c
mj_rne(m, d, 0, d->qfrc_bias);  // flg_acc=0，仅计算科里奥利项
```
这给出: `qfrc_bias = -C(q,v)`，其中 C 包含离心力和科里奥利力

#### qacc_smooth 计算（mj_fwdAcceleration）
```
qfrc_smooth = qfrc_passive - qfrc_bias + qfrc_applied + qfrc_actuator
qacc_smooth = M^(-1) * qfrc_smooth
```

这是无约束加速度。约束会修正最终的 qacc。

---

## 3. CRBA（质量矩阵计算）

**文件**: `/engine_core_smooth.c` (行 1514-1569)

### 算法概述
**Composite Rigid Body (CRB) 算法** - 递归计算质量矩阵的高效方法

### 关键步骤

```c
void mj_crb(const mjModel* m, mjData* d) {
```

1. **初始化复合惯性 (Composite Inertia)**
   ```c
   crb[i] = cinert[i]  // 复制每个身体的惯性张量（在 CoM 处）
   ```
   - `cinert[10*i..10*i+9]`: 10元素向量，表示 (质量, 3×3惯性张量)
   - 这些已在 `mj_comPos()` 中计算好

2. **向后递推：累积子树惯性**
   ```c
   for i from nbody-1 down to 1:
       crb[parent[i]] += crb[i]
   ```
   - 每个身体的 crb = 自身 + 所有后代的惯性
   - 这建立了复合刚体概念

3. **前向递推：计算质量矩阵**
   ```c
   for each dof i:
       buf = crb[body_i] * cdof[i]  // 惯性向量乘以运动自由度
       for ancestor j of i:
           M[i,j] += cdof[j] · buf
   ```

   **核心数学**:
   ```
   M[i,j] = Σ (c_dof_j)^T * I_composite[j] * c_dof_i
   ```
   其中求和遍历 j 和 i 的公共祖先

4. **稀疏性**
   - 只在 dof i 及其祖先间计算 M[i,j]
   - 链式结构导致带状矩阵
   - 时间复杂度: O(n²) （在树形链的情况）

### 输出数据结构
- `d->M[]`: 稀疏格式存储的质量矩阵
- `m->M_rowadr[]`: 行起始地址
- `m->M_colind[]`: 列索引
- `m->M_rownnz[]`: 每行非零元素数

### 关键量的定义

**运动自由度 (Motion DOF)**:
```c
cdof[6*j + 0..5]  // 6元素向量表示空间速度
                  // 前3个: 线性速度分量
                  // 后3个: 角速度分量
```

**复合惯性在 CoM 处**:
```c
cinert[10*i] = mass[i]
cinert[10*i+1..9] = 3×3惯性张量 (对角线化)
```

---

## 4. RNE（递归牛顿-欧拉算法）

**文件**: `/engine_core_smooth.c` (行 2068-2119)

### 算法目的
计算: `tau = M(q)*a + C(q,v)` 或仅 `C(q,v)`（科里奥利/离心项）

### 关键参数
- `flg_acc`: 如果=1，包含惯性项 M*a；如果=0，仅计算偏差力

### 算法步骤

#### 阶段1: 前向递推（从根到叶）
```c
for each body i from 1 to nbody-1:
    // 计算加速度
    cacc[i] = cacc[parent[i]] + cdof_dot[i]*qvel + cdof[i]*qacc
    
    // 计算力和力矩
    cfrc_body[i] = I[i] * cacc[i] + cross(cvel[i], I[i]*cvel[i])
```

**主要计算**:
- `cdof_dot * qvel`: 角加速度和线速度变化（运动学）
- `cdof * qacc`: 关节加速度对身体加速度的贡献
- `I * cacc`: 惯性力
- `cross(cvel, I*cvel)`: 心轴力（离心效应）

#### 阶段2: 后向递推（从叶到根）
```c
for each body i from nbody-1 down to 1:
    cfrc_body[parent[i]] += cfrc_body[i]
```
累积子树的力和力矩。

#### 阶段3: 投影到关节空间
```c
for each dof i:
    result[i] = cdof[i] · cfrc_body[body_i]
```

### 输出
```
result[] = {tau_0, tau_1, ..., tau_(nv-1)}
```
- 如果 `flg_acc=1`: τ = M*a + C(q,v)  （完整的广义力）
- 如果 `flg_acc=0`: τ = C(q,v)        （仅偏差项，用于 qfrc_bias）

### 应用场景

1. **qfrc_bias 计算** (mj_fwdVelocity)
   ```c
   mj_rne(m, d, 0, d->qfrc_bias);  // flg_acc=0
   ```
   给出科里奥利/离心力，用于前向动力学

2. **逆向动力学** (mj_inverse)
   ```c
   mj_rne(m, d, 0, d->qfrc_inverse);
   ```
   与约束力、被动力结合得到驱动力

---

## 5. 质量矩阵因式分解

**文件**: `/engine_core_smooth.c` (行 1644-1675)

### 目的
将质量矩阵 M 分解为 `M = L * D * L^T`，用于快速求解 `M*x = b`

### L'*D*L 分解
```c
void mj_factorM(const mjModel* m, mjData* d) {
    mju_copy(d->qLD, d->M, m->nC);
    mj_factorI(d->qLD, d->qLDiagInv, ...);
}
```

### 分解算法（Cholesky-like）
```c
for k from nv-1 down to 0:
    D[k,k] = M[k,k]
    
    // 更新三角形
    for i < k:
        L[i,k] = M[i,k] / D[k,k]
        for j <= i:
            M[i,j] -= L[i,k] * D[k,k] * L[i,k]
    
    L[k,k] /= D[k,k]
```

### 求解步骤（mj_solveLD）
```
1. 前向代入: x = inv(L') * x
2. 对角线缩放: x = inv(D) * x
3. 后向代入: x = inv(L) * x
```

时间复杂度: O(n²) （对于树形链）

---

## 6. 数据结构关键字段

### mjData 中的核心数组

| 字段 | 大小 | 含义 |
|------|------|------|
| `qpos[]` | nq | 广义坐标 |
| `qvel[]` | nv | 广义速度 |
| `qacc[]` | nv | 广义加速度 |
| `M[]` | nC | 质量矩阵（稀疏格式） |
| `qLD[]` | nC | L*D*L 分解因子 |
| `qLDiagInv[]` | nv | inv(D) 的对角线 |
| `cinert[]` | 10*nbody | CoM处的复合惯性 |
| `cdof[]` | 6*nv | 运动自由度向量 |
| `cvel[]` | 6*nbody | 身体速度（空间向量） |
| `cacc[]` | 6*nbody | 身体加速度 |
| `qfrc_bias[]` | nv | 偏差力（C项） |
| `qfrc_actuator[]` | nv | 执行器力 |
| `qfrc_constraint[]` | nv | 约束力 |

### mjModel 中的稀疏矩阵索引

| 字段 | 用途 |
|------|------|
| `M_rowadr[]` | nv个行起始地址 |
| `M_colind[]` | 列索引 |
| `M_rownnz[]` | 每行非零元素数 |
| `dof_parentid[]` | dof 的父 dof |
| `body_parentid[]` | 身体的父身体 |
| `dof_bodyid[]` | dof 所属身体 |

---

## 7. Rust 实现对应关系

### 建议的 Rust 模块映射

```rust
// kinematics.rs - 正向运动学
pub fn compute_kinematics(model: &Model, data: &mut Data)

// inertia.rs - CRBA和质量矩阵
pub fn compute_crb(model: &Model, data: &mut Data)
pub fn make_mass_matrix(model: &Model, data: &mut Data)

// rne.rs - RNE算法
pub fn rne(model: &Model, data: &mut Data, flg_acc: bool, result: &mut [f64])

// factor.rs - 质量矩阵因式分解
pub fn factor_m(model: &Model, data: &mut Data)
pub fn solve_m(model: &Model, data: &mut Data, x: &mut [f64], y: &[f64])

// integrators.rs - 积分器
pub fn rk4_step(model: &Model, data: &mut Data)
pub fn euler_step(model: &Model, data: &mut Data)

// forward.rs - 前向动力学
pub fn forward_dynamics(model: &Model, data: &mut Data)
```

### 关键常数对照

| MuJoCo | 含义 | Rust替代 |
|--------|------|---------|
| mjMINVAL | 最小值防护 (1e-19) | 常量 |
| mjMINSCALE | 最小缩放 (1e-8) | 常量 |
| nv | 速度维度 | model.nv |
| nq | 位置维度 | model.nq |
| na | 激活维度 | model.na |
| nC | 质量矩阵非零数 | model.nC |

---

## 8. 精度和数值稳定性

### 重要实现细节

1. **对角线最小值检查**
   ```c
   if (qLD[Madr_kk] < mjMINVAL) {
       qLD[Madr_kk] = mjMINVAL;
   }
   ```

2. **四元数规范化**
   - 每步运动学后都要规范化四元数
   - 防止累积舍入误差

3. **空间向量运算**
   - 6元素向量表示 (线速度, 角速度)
   - 使用特殊的叉积和惯性乘法

4. **稀疏矩阵存储**
   - 压缩行存储格式 (CRS)
   - 仅存储父-子关联的 DOF 间的非零元

### 性能优化

1. **缓存局部性**
   - CRBA 按照身体树顺序访问
   - RNE 使用两次遍历（前向+后向）

2. **避免重复计算**
   - cinert 在 mj_comPos() 中一次性计算
   - cdof 在 mj_comPos() 中一次性计算

3. **线程并行化**
   - 约束求解可以并行化（岛屿结构）
   - 碰撞和惯性矩阵可并行计算

---

## 总结：关键算法流程

### 典型时间步的计算流程

```
mj_step()
├─ mj_checkPos()
├─ mj_checkVel()
├─ mj_forward()
│  ├─ mj_fwdPosition()
│  │  ├─ mj_kinematics()         # 计算 xpos, xmat
│  │  ├─ mj_comPos()             # 计算 cinert, cdof
│  │  ├─ mj_crb()                # CRBA → M
│  │  └─ mj_factorM()            # L*D*L分解
│  │
│  ├─ mj_fwdVelocity()
│  │  └─ mj_rne(flg_acc=0)       # 计算 qfrc_bias
│  │
│  ├─ mj_fwdActuation()
│  │  └─ 计算 qfrc_actuator
│  │
│  ├─ mj_fwdAcceleration()
│  │  └─ mj_solveM()             # 求解 qacc_smooth = M^(-1)*qfrc_smooth
│  │
│  └─ mj_fwdConstraint()
│     └─ 约束求解器 → 最终 qacc
│
└─ 积分器 (mj_RK4 或 mj_Euler)
   ├─ 更新位置、速度、激活
   └─ 推进时间
```

### 关键方程对照

| 方程 | MuJoCo实现 | 功能 |
|------|-----------|------|
| M*a = τ - C | mj_solveM() | 前向动力学 |
| τ = M*a + C | mj_rne(1) | 逆向动力学 |
| C = C(q,v) | mj_rne(0) | 偏差力 |
| M = CRBA() | mj_crb() | 质量矩阵计算 |
| M = L*D*L^T | mj_factorM() | 因式分解 |

