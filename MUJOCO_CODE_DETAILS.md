# MuJoCo 核心代码实现细节

## 文件位置速查表

| 功能 | 文件 | 函数 | 行号范围 |
|------|------|------|---------|
| RK4积分器 | engine_forward.c | mj_RungeKutta | 904-992 |
| 前向动力学主入口 | engine_forward.c | mj_forward | 1155 |
| 位置计算 | engine_forward.c | mj_fwdPosition | 129-185 |
| 速度计算 | engine_forward.c | mj_fwdVelocity | 189-228 |
| 加速度计算 | engine_forward.c | mj_fwdAcceleration | 550-561 |
| 约束处理 | engine_forward.c | mj_fwdConstraint | 693-782 |
| CRBA算法 | engine_core_smooth.c | mj_crb | 1514-1569 |
| 质量矩阵计算 | engine_core_smooth.c | mj_makeM | 1572-1578 |
| 质量矩阵分解 | engine_core_smooth.c | mj_factorM | 1645-1650 |
| 矩阵分解核心 | engine_core_smooth.c | mj_factorI | 1654-1675 |
| RNE算法 | engine_core_smooth.c | mj_rne | 2068-2119 |
| 正向运动学 | engine_core_smooth.c | mj_kinematics | 37-177 |
| CoM相关计算 | engine_core_smooth.c | mj_comPos | 181-259 |
| Euler积分器 | engine_forward.c | mj_Euler | 885-887 |
| 隐式积分器 | engine_forward.c | mj_implicit | 1063-1065 |

---

## 1. 正向运动学详解 (mj_kinematics)

### 源代码位置
`/engine_core_smooth.c` 行 37-177

### 核心步骤

```c
void mj_kinematics(const mjModel* m, mjData* d) {
    // 1. 初始化世界
    d->xpos[0..2] = 0        // 世界位置在原点
    d->xquat[0..3] = [1,0,0,0]  // 身份四元数
    d->xmat[0..8] = I_3x3    // 单位矩阵
    
    // 2. 前向传播（body 1 到 nbody-1）
    for i in 1..nbody-1:
        // a) 获取身体的关节信息
        jntadr = body_jntadr[i]
        jntnum = body_jntnum[i]
        
        // b) 处理自由关节
        if jntnum==1 and jnt_type[jntadr]==mjJNT_FREE:
            xpos[i] = qpos[qposadr[jntadr]:qposadr[jntadr]+3]
            xquat[i] = qpos[qposadr[jntadr]+3:qposadr[jntadr]+7]
        
        // c) 处理受限关节
        else:
            pid = body_parentid[i]
            
            // 继承父身体的位置和方向
            xpos[i] = xpos[pid] + R(xquat[pid]) @ body_pos[i]
            xquat[i] = xquat[pid] * body_quat[i]  // 四元数乘法
            
            // 对每个关节应用变换
            for j in 0..jntnum-1:
                jid = jntadr + j
                
                // 获取关节轴和锚点（在全局坐标系中）
                xaxis[jid] = R(xquat[i]) @ jnt_axis[jid]
                xanchor[jid] = xpos[i] + R(xquat[i]) @ jnt_pos[jid]
                
                // 根据关节类型应用变换
                match jnt_type[jid]:
                    mjJNT_SLIDE:
                        // 滑动关节：沿轴平移
                        offset = qpos[qposadr[jid]] - qpos0[jid]
                        xpos[i] += offset * xaxis[jid]
                    
                    mjJNT_HINGE:
                        // 铰链关节：绕轴旋转
                        angle = qpos[qposadr[jid]] - qpos0[jid]
                        q_local = axis_angle_to_quat(xaxis[jid], angle)
                        xquat[i] = xquat[i] * q_local
                        
                        // 修正旋转中心偏移
                        xpos[i] = xanchor[jid] - R(xquat[i]) @ jnt_pos[jid]
                    
                    mjJNT_BALL:
                        // 球形关节：任意方向旋转
                        q_local = normalize(qpos[qposadr[jid]:qposadr[jid]+4])
                        xquat[i] = xquat[i] * q_local
                        xpos[i] = xanchor[jid] - R(xquat[i]) @ jnt_pos[jid]
        
        // d) 规范化四元数（防止舍入误差）
        xquat[i] = normalize(xquat[i])
        
        // e) 计算旋转矩阵（用于快速运算）
        xmat[i] = quat_to_mat3(xquat[i])  // 9个元素
    
    // 3. 计算身体的惯性框架
    for i in 1..nbody-1:
        // 从局部坐标变换到全局坐标
        xipos[i] = xpos[i] + R(xmat[i]) @ body_ipos[i]
        ximat[i] = xmat[i] @ body_iquat_to_mat[i]
    
    // 4. 计算几何体位置
    for i in 0..ngeom-1:
        bodyid = geom_bodyid[i]
        geom_xpos[i] = xpos[bodyid] + R(xmat[bodyid]) @ geom_pos[i]
        geom_xmat[i] = xmat[bodyid] @ geom_quat_to_mat[i]
}
```

### 关键数据流
```
qpos[] ──┐
         ├─> mj_kinematics() ──> xpos[], xquat[], xmat[]
body[]   │
         └─> 用于碰撞检测和传感器
```

---

## 2. CRBA 详细分析

### 源代码位置
`/engine_core_smooth.c` 行 1514-1569

### 前提条件
- `cinert[]` 已计算（在 mj_comPos 中）
- `cdof[]` 已计算（在 mj_comPos 中）

### 伪代码

```c
void mj_crb(const mjModel* m, mjData* d) {
    int nv = m->nv;
    int nbody = m->nbody;
    
    // =========== STEP 1: 初始化复合惯性 ===========
    // 复制每个身体的惯性（在其CoM处）
    for i in 0..nbody-1:
        crb[10*i .. 10*i+9] = cinert[10*i .. 10*i+9]
    
    // =========== STEP 2: 向后递推（树根到叶子） ===========
    // 累积子树惯性到父身体
    for i from nbody-1 down to 1:
        pid = body_parentid[i]
        if pid > 0:
            crb[10*pid] += crb[10*i]         // 质量
            crb[10*pid+1..9] += crb[10*i+1..9]  // 惯性张量
    
    // 结果：crb[i] 包含身体 i 及其所有后代的惯性
    
    // =========== STEP 3: 清空质量矩阵 ===========
    M[] = 0
    
    // =========== STEP 4: 前向递推计算质量矩阵 ===========
    for i in 0..nv-1:
        // 获取该dof的地址和身体
        int bodyid = dof_bodyid[i]
        int adr = M_rowadr[i]
        int diag_idx = M_rownnz[i] - 1  // 对角线在行末
        
        // 对于简单dof（e.g., 弹簧-阻尼器）
        if dof_simplenum[i]:
            M[adr] = dof_M0[i]
            continue
        
        // 初始化对角线（加入电枢惯性）
        M[adr + diag_idx] = dof_armature[i]
        
        // ========== 关键步骤：计算 M[i,:] ==========
        
        // 1. 计算 buf = crb[body_i] * cdof[i]
        //    这是复合惯性在该dof方向上的投影
        buf[0..5] = inertia_vector_multiply(
            crb[10*bodyid .. 10*bodyid+9],
            cdof[6*i .. 6*i+5]
        )
        
        // 2. 对于该dof的所有祖先dof，计算M[i,j]
        int j = i
        int M_addr = adr + diag_idx
        
        while j >= 0:
            // M[i,j] += cdof[j] · buf
            M[M_addr] += dot_product(
                cdof[6*j .. 6*j+5],
                buf[0..5],
                6
            )
            
            // 移动到父dof
            j = dof_parentid[j]
            M_addr++  // 稀疏格式中向前移动
}
```

### 关键数据操作

#### 惯性向量乘法 (mju_mulInertVec)
```
输入: I (10元) = [mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, (padding)]
      v (6元) = [线速度 v_lin, 角速度 v_ang]

输出: out (6元) = [m*v_lin + cross(I_lower*v_ang, v_lin),
                    I_upper*v_ang + cross(v_lin, m*v_lin)]
```

这实现了空间惯性对速度的作用。

#### 稀疏矩阵寻址

```
行i的非零元存储在: M[M_rowadr[i] .. M_rowadr[i] + M_rownnz[i] - 1]

对于链式系统（每个dof只有一个父dof）：
- 行i包含: M[i,i], M[i, parent[i]], M[i, parent[parent[i]]], ...

列索引由 M_colind[] 给出（或可从 dof_parentid[] 推导）
```

### 输出特性

- M 是**对称正定矩阵**（SPD）
- **带状**（bandwidth ≈ 树的深度）
- **稀疏**（对于链式结构，非零元 ~ O(n²)）
- 直接用于因式分解

---

## 3. RNE 算法详细流程

### 源代码位置
`/engine_core_smooth.c` 行 2068-2119

### 关键数据结构

```c
// 输入
d->qpos[]    // 广义坐标（位置）
d->qvel[]    // 广义速度
d->qacc[]    // 广义加速度（如果 flg_acc=1）
d->cinert[]  // 复合惯性（在CoM处）
d->cdof[]    // 运动自由度向量
d->cdof_dot[] // 运动自由度时间导数

// 中间计算
d->cvel[]    // 身体速度（6维）
d->cacc[]    // 身体加速度（6维）

// 输出
result[]     // 广义力（tau）
```

### 伪代码

```c
void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result) {
    int nbody = m->nbody;
    int nv = m->nv;
    
    // ============ PHASE 1: 初始化 ============
    // 身体0（世界）的加速度 = -重力
    loc_cacc[0..5] = [0, 0, 0, -g_x, -g_y, -g_z]
    
    // ============ PHASE 2: 前向递推（身体1到nbody-1） ============
    // 计算每个身体的加速度和内力
    for i from 1 to nbody-1:
        int bda = body_dofadr[i]  // 该身体第一个dof的地址
        int ndof = body_dofnum[i] // 该身体的dof数
        
        // -------- 子步骤 1: 累积加速度 --------
        // cacc[i] = cacc[parent[i]] + 由于速度变化的加速度 + 由于qacc的加速度
        
        // a) 速度引起的加速度（科里奥利/离心）
        tmp = cdof_dot @ qvel[bda:bda+ndof]  // cdof_dot 是 cdof 对时间的导数
        loc_cacc[6*i:6*i+6] = loc_cacc[6*parent[i]:6*parent[i]+6] + tmp
        
        // b) qacc引起的加速度
        if flg_acc:
            tmp = cdof @ qacc[bda:bda+ndof]
            loc_cacc[6*i:6*i+6] += tmp
        
        // -------- 子步骤 2: 计算身体上的力和力矩 --------
        // 使用 F = I*a + omega x (I*omega) （离心项）
        
        // 惯性力
        cfrc_body[6*i:6*i+6] = cinert[10*i] * loc_cacc[6*i:6*i+6]
        
        // 离心/科里奥利修正
        tmp = cinert[10*i] * cvel[6*i:6*i+6]
        tmp1 = cross_product(cvel[6*i:6*i+6], tmp)  // omega x (I*omega)
        cfrc_body[6*i:6*i+6] += tmp1
    
    // ============ PHASE 3: 后向递推（nbody-1到1） ============
    // 累积子树的力到父身体
    for i from nbody-1 down to 1:
        pid = body_parentid[i]
        if pid > 0:
            cfrc_body[6*pid:6*pid+6] += cfrc_body[6*i:6*i+6]
    
    // ============ PHASE 4: 投影到关节空间 ============
    // 将身体力投影回广义坐标
    for i from 0 to nv-1:
        bodyid = dof_bodyid[i]
        result[i] = dot_product(
            cdof[6*i:6*i+6],
            cfrc_body[6*bodyid:6*bodyid+6],
            6
        )
}
```

### 关键运算解释

#### 1. cdof_dot 的作用

`cdof_dot` 表示运动自由度对时间的导数。它来自于:

```
d(cdof)/dt = d/dt[R(q) * [ 0, r_offset x; 0, R(q)^T*axis ]]
```

这包含了两部分：
- 旋转速率对自由度的影响
- 偏移点随身体运动而产生的额外速度

#### 2. 身体力的计算

```
cfrc_body[i] = I_composite[i] * cacc[i] + (cvel[i] x (I_composite[i] * cvel[i]))
```

第一项是惯性力（F=ma），第二项是向心加速度。

#### 3. 稀疏求和

在后向递推中，每个身体只加到其唯一的父身体，利用树结构的稀疏性。

---

## 4. 质量矩阵因式分解

### 源代码位置
`/engine_core_smooth.c` 行 1645-1675

### 算法：L'*D*L 分解

分解矩阵 M 为 `M = L * D * L^T`，其中：
- L: 单位下三角矩阵
- D: 对角矩阵

### 伪代码

```c
void mj_factorI(mjtNum* mat, mjtNum* diaginv, int nv,
                const int* rownnz, const int* rowadr, const int* colind) {
    
    // ========== 因式分解：从最后一行向前 ==========
    for k from nv-1 down to 0:
        // 获取第k行的地址和相关信息
        int start = rowadr[k]
        int diag_idx = rownnz[k] - 1  // 对角线在末尾
        int end = start + diag_idx
        
        // D[k,k] = M[k,k]
        mjtNum D_kk = mat[end]
        mjtNum invD = 1.0 / D_kk
        if (diaginv) diaginv[k] = invD
        
        // -------- 更新第k行上面的行 --------
        // 对于所有 i < k 且 M[i,k] != 0:
        //   M[i,0..i] -= M[i,k]/D[k,k] * M[k,0..i]
        
        for adr from end-1 down to start:
            int i = colind[adr]  // 列号 = 该非零元对应的行
            
            // L[i,k] = M[i,k] / D[k,k]
            mjtNum L_ik = mat[adr] * invD
            
            // 更新第i行
            int i_start = rowadr[i]
            int i_nnz = rownnz[i]
            
            // M[i,0..i] -= L[i,k] * D[k,k] * L[i,k]^T * M[k,0..i]^T
            //          = M[i,0..i] - L[i,k]^2 * D[k,k] * ...
            // 实际上是 M[i,0..i] -= L[i,k]/invD * M[k,0..i]
            
            for j from 0 to i_nnz-1:
                mat[i_start + j] -= mat[start + j] * L_ik * invD
        }
        
        // -------- 规范化第k行 --------
        // L[k,0..k-1] = M[k,0..k-1] / D[k,k]
        for adr from start to end-1:
            mat[adr] *= invD  // 除以 D[k,k]
}
```

### 求解过程（L*D*L^T 回代）

```c
// 给定 L*D*L^T = M，求解 M*x = y for x

// ========== STEP 1: 前向代入（解 L^T*z = y） ==========
for i from nv-1 down to 0:
    if x[i] != 0:  // 稀疏性优化
        int adr_ij = M_rowadr[i] + 1
        int j = dof_parentid[i]
        
        while j >= 0:
            x[j] -= L[i,j] * x[i]  // x[j] -= M[i,j]/D[i,i] * x[i]
            j = dof_parentid[j]

// ========== STEP 2: 对角线缩放（z = inv(D) * z） ==========
for i from 0 to nv-1:
    x[i] *= diaginv[i]

// ========== STEP 3: 后向代入（解 L*w = z） ==========
for i from 0 to nv-1:
    int adr_ij = M_rowadr[i] + 1
    int j = dof_parentid[i]
    
    while j >= 0:
        x[i] -= L[i,j] * x[j]  // x[i] -= M[i,j]/D[i,i] * x[j]
        j = dof_parentid[j]
```

### 时间复杂度
- 因式分解: O(n²) （在树形链的情况）
- 回代: O(n²) （最坏情况）
- 对于深树: 接近 O(n)

---

## 5. RK4 积分器详细分析

### 源代码位置
`/engine_forward.c` 行 890-992

### RK4 系数

```
Butcher tableau:
    0   |  0      0      0
  1/2   | 1/2     0      0
  1/2   |  0     1/2     0
    1   |  0      0      1
    ---|--------------------
        | 1/6    1/3    1/3    1/6

其中：
A = [1/2,  0,   0;
     0,   1/2,  0;
     0,    0,   1]

B = [1/6, 1/3, 1/3, 1/6]
```

### 伪代码

```c
void mj_RungeKutta(const mjModel* m, mjData* d, int N=4) {
    // N 是积分阶数（目前仅支持 N=4）
    
    int nv = m->nv, nq = m->nq, na = m->na
    mjtNum h = m->opt.timestep
    mjtNum time_init = d->time
    
    // ========== 初始化 ==========
    // 预计算 RK 系数中的累加时间点
    C[0] = 1/2        // 第一个中间点 t + h/2
    C[1] = 1/2        // 第二个中间点 t + h/2
    C[2] = 1          // 最后一个中间点 t + h
    
    T[0] = t_init + C[0]*h
    T[1] = t_init + C[1]*h
    T[2] = t_init + C[2]*h
    
    // ========== 阶段 0（初始）==========
    // 使用已计算的加速度（调用mj_step前已通过mj_forward计算）
    X[0] = (qpos, qvel, act) at t_init
    F[0] = (qacc, act_dot) at t_init
    
    // ========== 阶段 1-3（RK中间点）==========
    for i in 1..3:
        // 计算增量（权重求和）
        dX = 0
        for j in 0..i-1:
            dX.vel += A[i,j] * X[j].vel        // 速度累加
            dX.act_dot += A[i,j] * F[j].act_dot // 激活导数累加
            // 注意：位置增量在下面处理
        
        // 更新状态到中间点
        X[i].qpos = X[0].qpos + mj_integratePos(X[0..i-1].qvel, A[i,*], h)
        X[i].qvel = X[0].qvel + dX.vel * h
        X[i].act = X[0].act + dX.act_dot * h
        
        // 在中间点计算导数
        d->qpos = X[i].qpos
        d->qvel = X[i].qvel
        d->act = X[i].act
        d->time = T[i-1]
        
        mj_forwardSkip(m, d, mjSTAGE_NONE, 1)  // 计算新的 qacc, act_dot
        
        F[i] = (d->qacc, d->act_dot)
    
    // ========== 最终更新 ==========
    // 使用 B 系数（权重）汇总
    dX_final.vel = 0
    dX_final.act_dot = 0
    for j in 0..3:
        dX_final.vel += B[j] * F[j].qacc
        dX_final.act_dot += B[j] * F[j].act_dot
    dX_final.pos = Σ B[j] * X[j].qvel  // 位置增量来自速度
    
    // 应用最终更新
    d->time = time_init
    d->qpos = X[0].qpos + mj_integratePos(dX_final.pos, 1.0, h)
    d->qvel = X[0].qvel + dX_final.vel * h
    d->act = X[0].act + dX_final.act_dot * h
    d->time += h
    
    // 保存加速度用于下一步 warmstart
    d->qacc_warmstart = d->qacc
}
```

### 关键特性

1. **半隐式积分**
   - 位置：`q_{k+1} = q_k + h * v_weighted`
   - 速度：`v_{k+1} = v_k + h * a_weighted`
   - 激活：`a_{k+1} = a_k + h * adot_weighted`

2. **四元数处理**
   - `mj_integratePos()` 处理旋转
   - 四元数用轴角或四元数形式增量表示
   - 每步后规范化四元数

3. **递归调用**
   - 各阶段调用 `mj_forwardSkip()`
   - 完整的正向动力学（不仅是加速度）
   - 允许变参数动力学（例如接触）

---

## 6. 前向动力学完整流程

### 顶层函数

```c
void mj_step(const mjModel* m, mjData* d) {
    // 第一阶段：检查和初步计算
    mj_checkPos(m, d)    // 检查位置的NaN/Inf
    mj_checkVel(m, d)    // 检查速度的NaN/Inf
    mj_forward(m, d)     // 主要动力学计算
    mj_checkAcc(m, d)    // 检查加速度的NaN/Inf
    
    // 可选：验证前向/逆向一致性
    if mjENABLED(mjENBL_FWDINV):
        mj_compareFwdInv(m, d)
    
    // 第二阶段：积分
    switch m->opt.integrator:
        case mjINT_EULER:
            mj_Euler(m, d)
        case mjINT_RK4:
            mj_RungeKutta(m, d, 4)
        case mjINT_IMPLICIT:
            mj_implicit(m, d)
        ...
}
```

### 前向动力学主函数

```c
void mj_forward(const mjModel* m, mjData* d) {
    // 调用 mj_forwardSkip，不跳过任何阶段
    mj_forwardSkip(m, d, mjSTAGE_NONE, 0)
}

void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
    // ========== 位置相关 ==========
    if skipstage < mjSTAGE_POS:
        mj_fwdPosition(m, d)
            // 包含：运动学、质量矩阵、因式分解
    
    // ========== 速度相关 ==========
    if skipstage < mjSTAGE_VEL:
        mj_fwdVelocity(m, d)
            // 包含：RNE(flg_acc=0) 用于 qfrc_bias
    
    // ========== 执行器 ==========
    mj_fwdActuation(m, d)
        // 计算 qfrc_actuator，受 ctrl 和 act 驱动
    
    // ========== 加速度 ==========
    mj_fwdAcceleration(m, d)
        // 汇总力，求解 M*qacc_smooth = qfrc_smooth
    
    // ========== 约束 ==========
    mj_fwdConstraint(m, d)
        // 约束求解，得到最终 qacc 和 qfrc_constraint
}
```

### mj_fwdPosition 详解

```c
void mj_fwdPosition(const mjModel* m, mjData* d) {
    // -------- 第1部分：运动学和质量相关 --------
    mj_kinematics(m, d)      // xpos, xmat, xquat
    mj_comPos(m, d)          // cinert, cdof, subtree_com
    mj_camlight(m, d)        // 相机/灯光（不影响动力学）
    mj_flex(m, d)            // 柔性体（如果有）
    mj_tendon(m, d)          // 肌腱长度和力矩臂
    
    // -------- 第2部分：质量矩阵和碰撞（可并行） --------
    mj_makeM(m, d)           // CRBA 计算质量矩阵
    mj_factorM(m, d)         // L*D*L' 分解
    mj_collision(m, d)       // 碰撞检测和联系人生成
    
    // -------- 第3部分：约束相关 --------
    mj_makeConstraint(m, d)  // 生成约束雅可比和偏差
    mj_island(m, d)          // 识别独立约束岛
    mj_transmission(m, d)    // 传输（肌腱等）
    mj_projectConstraint(m, d) // 约束投影（如果需要）
}
```

### mj_fwdVelocity 详解

```c
void mj_fwdVelocity(const mjModel* m, mjData* d) {
    // -------- 第1部分：速度相关 --------
    // 这些已在 mj_fwdPosition 的 mj_comVel 中计算
    // （此处重复调用以确保一致性）
    mj_comVel(m, d)  // 身体速度和加速度相关导数
    
    // -------- 第2部分：被动力 --------
    mj_passive(m, d) // 阻尼、摩擦等被动元素
    
    // -------- 第3部分：约束参考 --------
    mj_referenceConstraint(m, d)
    
    // -------- 第4部分：科里奥利/离心力 --------
    // 最关键：计算 qfrc_bias = C(q, v)
    mj_rne(m, d, 0, d->qfrc_bias)  // flg_acc = 0，不包含惯性
    
    // -------- 第5部分：肌腱偏差 --------
    mj_tendonBias(m, d, d->qfrc_bias)
}
```

### mj_fwdAcceleration 详解

```c
void mj_fwdAcceleration(const mjModel* m, mjData* d) {
    // 汇总所有力
    // qfrc_smooth = qfrc_applied + qfrc_actuator - qfrc_bias - qfrc_passive + xfrc_applied
    
    // 实现（重新排列）：
    qfrc_smooth = qfrc_passive - qfrc_bias  // qfrc_bias 已是负的
    qfrc_smooth += qfrc_applied
    qfrc_smooth += qfrc_actuator
    xfrc_accumulate(m, d, qfrc_smooth)  // 加入施加的外力
    
    // 求解无约束加速度
    // qacc_smooth = M^{-1} * qfrc_smooth
    mj_solveM(m, d, d->qacc_smooth, qfrc_smooth, 1)
    
    // 如果有约束，qacc 会进一步修正
}
```

### mj_fwdConstraint 详解

```c
void mj_fwdConstraint(const mjModel* m, mjData* d) {
    // -------- 如果没有约束，直接返回 --------
    if d->nefc == 0:
        qacc = qacc_smooth
        return
    
    // -------- 计算约束加速度偏差 --------
    efc_b = Jac * qacc_smooth - aref
    // 这是在非约束加速度下的约束违反
    
    // -------- Warmstart：使用上一步的力估计 --------
    // 对约束求解器进行初始化
    
    // -------- 约束求解 --------
    // 选择求解器（PGS/CG/Newton）
    if use_islands and support_islands:
        // 在独立岛上并行求解
        for each island:
            solve_island(...)
    else:
        // 全局求解
        switch m->opt.solver:
            case PGS:
                mj_solPGS(m, d, iterations)
            case CG:
                mj_solCG(m, d, iterations)
            case NEWTON:
                mj_solNewton(m, d, iterations)
    
    // --------可选：不滑移求解器 --------
    if m->opt.noslip_iterations > 0:
        mj_solNoSlip(m, d, noslip_iterations)
}
```

---

## 7. 数据流总结

### 一个完整时间步的数据流

```
初始状态: (qpos, qvel, act)
    ↓
mj_step()
    ├─ mj_forward()
    │  ├─ mj_fwdPosition()
    │  │  ├─ mj_kinematics()      → (xpos, xmat, xquat)
    │  │  ├─ mj_comPos()          → (cinert, cdof)
    │  │  ├─ mj_makeM()           → M (CRBA)
    │  │  └─ mj_factorM()         → (qLD, qLDiagInv)
    │  │
    │  ├─ mj_fwdVelocity()
    │  │  ├─ mj_comVel()          → (cvel)
    │  │  └─ mj_rne(0)            → qfrc_bias
    │  │
    │  ├─ mj_fwdActuation()       → qfrc_actuator
    │  │
    │  ├─ mj_fwdAcceleration()
    │  │  ├─ 汇总力              → qfrc_smooth
    │  │  └─ mj_solveM()          → qacc_smooth
    │  │
    │  └─ mj_fwdConstraint()      → (qacc, qfrc_constraint, efc_force)
    │
    └─ 积分器 (Euler/RK4/Implicit)
        ├─ 更新位置和速度
        └─ 推进时间
            ↓
新状态: (qpos', qvel', act')
```

---

## 8. 关键优化技巧

### 1. 稀疏性利用
```
质量矩阵 M 在树形链中：
- 行i有 depth[i] 个非零元（从i到根）
- 总非零元数约 n*depth
- 对深树，depth ~ log(n)，对链，depth ~ n
```

### 2. 缓存重用
```
- cinert 和 cdof 在位置变化时计算，之后重复使用
- 不重新计算运动学
```

### 3. 条件跳过
```
- mjSTAGE_NONE: 完整计算
- mjSTAGE_POS: 跳过位置计算（用于某些积分器）
- mjSTAGE_VEL: 跳过速度和位置计算
```

### 4. 并行化
```
- CRBA 和碰撞可并行
- 约束岛独立求解
- CG/Newton 迭代可并行（受限于通讯）
```

---

