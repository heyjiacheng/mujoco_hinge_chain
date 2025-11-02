# MuJoCo Dynamics Computation: Complete Technical Analysis

## Executive Summary

This document provides an extremely detailed analysis of how MuJoCo computes dynamics using the RNE (Recursive Newton-Euler) algorithm and related functions. The analysis covers three core areas:

1. **mj_comVel()** - Computing body velocities in spatial coordinates
2. **mj_rne()** - Complete RNE algorithm for inverse dynamics
3. **mj_passive()** - Passive forces (damping and springs)

The implementation uses sophisticated spatial algebra with 6D vectors representing both motion (velocity, angular velocity) and forces (torque, force).

---

## SECTION 1: Spatial Algebra Fundamentals

### 1.1 Spatial Vector Representation

MuJoCo uses **6D spatial vectors** with a specific layout:
```
Vector[6] = [angular_component[3], linear_component[3]]
            = [ω_x, ω_y, ω_z, v_x, v_y, v_z]
```

**Key distinction**: This is NOT the standard screw notation. MuJoCo stores rotation first, translation second.

**Data structures**:
- `d->cvel[6*i]` : Spatial velocity of body i (in composite frame, centered at body's COM)
- `d->cdof[6*i]` : Motion subspace (6D basis vector for each DOF)
- `d->cdof_dot[6*i]` : Time derivative of motion subspace (Coriolis/centrifugal term)
- `d->cinert[10*i]` : Spatial inertia matrix (10 elements = symmetric 6x6 matrix in triangular form)
- `d->cacc[6*i]` : Spatial acceleration
- `d->cfrc_body[6*i]` : Spatial force acting on body

---

## SECTION 2: mj_comVel() - Body Velocity Computation

### Location
- **File**: `engine_core_smooth.c`
- **Lines**: 1932-1997

### Function Signature
```c
void mj_comVel(const mjModel* m, mjData* d) {
```

### High-Level Algorithm

The function performs a **forward pass** over all bodies (world to leaves) to compute:
1. **cvel**: Spatial velocity of each body in composite frame (at COM)
2. **cdof_dot**: Time derivative of motion subspace (Coriolis/centrifugal effects)

### Detailed Step-by-Step Walkthrough

#### Step 1: Initialize world velocity (Lines 1935-1936)
```c
// Line 1936
mju_zero(d->cvel, 6);
```
The world body has zero velocity. This is the base case for the recursion.

#### Step 2: Forward pass over bodies (Lines 1939-1996)

**Loop structure**: `for (int i=1; i < nbody; i++)`

For each body i:

##### 2a. Get parent velocity (Lines 1943-1945)
```c
int bda = m->body_dofadr[i];                    // First DOF address for this body
mjtNum cvel[6];
mju_copy(cvel, d->cvel+6*m->body_parentid[i], 6);  // cvel = parent's spatial velocity
```

This copies the parent body's velocity as the starting point. This is the **propagation of velocity** up the kinematic tree.

##### 2b. Handle different joint types (Lines 1948-1991)

The function treats different joint types differently because they have different characteristics for velocity and acceleration:

**Case 1: FREE Joint (Lines 1955-1965)**

```c
case mjJNT_FREE:
    // Line 1957: cdofdot = 0 (translations don't have Coriolis terms)
    mju_zero(cdofdot, 18);
    
    // Line 1960: Update velocity with translational DOFs
    // cvel += cdof * qvel (for 3 translational DOFs)
    mju_mulDofVec(tmp, d->cdof+6*bda, d->qvel+bda, 3);
    mju_addTo(cvel, tmp, 6);
    
    j += 3;  // Skip the translational part in next iteration
    mjFALLTHROUGH;  // Continue to BALL case for rotational DOFs
```

**Mathematical meaning**:
```
cvel = cvel_parent + Σ(cdof[k] * qvel[k])   for k in translational DOFs
cdof_dot = 0   (no Coriolis terms for translation)
```

**Case 2: BALL Joint (Lines 1967-1979)**

```c
case mjJNT_BALL:
    // Lines 1969-1971: Compute cdofdot using cross-product
    for (int k=0; k < 3; k++) {
        mju_crossMotion(cdofdot+6*(j+k), cvel, d->cdof+6*(bda+j+k));
    }
    
    // Line 1974: Update velocity with rotational DOFs
    // cvel += cdof * qvel (for 3 rotational DOFs)
    mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 3);
    mju_addTo(cvel, tmp, 6);
    
    j += 2;  // Adjust loop counter (3 DOFs total)
```

**Mathematical meaning**:
```
cdof_dot[k] = cvel × cdof[k]   (spatial cross-product for motion)
cvel = cvel_parent + Σ(cdof[k] * qvel[k])   for k in rotational DOFs
```

**Key insight**: `mju_crossMotion()` computes the Lie bracket [cvel, cdof], which represents how the motion subspace changes in the parent's velocity frame.

**Case 3: HINGE/SLIDE (Default) (Lines 1981-1990)**

```c
default:
    // Line 1985: Compute cdofdot
    mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));
    
    // Line 1988: Update velocity with this DOF
    mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
    mju_addTo(cvel, tmp, 6);
```

**Code comment (Lines 1982-1984)** explains an important numerical subtlety:
```c
// "in principle we should use the new velocity to compute cdofdot,
//  but it makes no difference because crossMotion(cdof, cdof) = 0,
//  and using the old velocity may be more accurate numerically"
```

This is why the code computes `cdofdot = cvel_parent × cdof`, not `cvel_new × cdof`.

##### 2c. Store results (Lines 1993-1995)
```c
mju_copy(d->cvel+6*i, cvel, 6);           // Store body's spatial velocity
mju_copy(d->cdof_dot+6*bda, cdofdot, 6*dofnum);  // Store time derivatives
```

### Key Spatial Algebra Operations

#### mju_mulDofVec(res, dof_matrix, vec, n)
**Location**: `engine_util_spatial.c`, lines 461-469

```c
void mju_mulDofVec(mjtNum* res, const mjtNum* dof, const mjtNum* vec, int n) {
  if (n == 1) {
    mju_scl(res, dof, vec[0], 6);  // Single DOF: res = dof * vec[0]
  } else if (n <= 0) {
    mju_zero(res, 6);
  } else {
    mju_mulMatTVec(res, dof, vec, n, 6);  // Multiple DOFs: res = dof^T @ vec
  }
}
```

**What it does**: Computes the contribution of a joint's DOFs to the spatial velocity.
```
Σ(cdof[k] * qvel[k]) for k in [0, n)
```

#### mju_crossMotion(res, vel, v)
**Location**: `engine_util_spatial.c`, lines 370-381

This computes the Lie bracket for motion vectors: **res = vel × v**

```c
void mju_crossMotion(mjtNum res[6], const mjtNum vel[6], const mjtNum v[6]) {
  // res = [ω, v]_spatial = [-ω × ω_v, -ω × v_v - ω_v × v_ω]
  
  // Angular part: -ω × ω_v (where ω = vel[0:3], ω_v = v[0:3])
  res[0] = -vel[2]*v[1] + vel[1]*v[2];
  res[1] =  vel[2]*v[0] - vel[0]*v[2];
  res[2] = -vel[1]*v[0] + vel[0]*v[1];
  
  // Linear part first half: -ω × v_v
  res[3] = -vel[2]*v[4] + vel[1]*v[5];
  res[4] =  vel[2]*v[3] - vel[0]*v[5];
  res[5] = -vel[1]*v[3] + vel[0]*v[4];
  
  // Linear part second half: -ω_v × v_ω
  res[3] += -vel[5]*v[1] + vel[4]*v[2];
  res[4] +=  vel[5]*v[0] - vel[3]*v[2];
  res[5] += -vel[4]*v[0] + vel[3]*v[1];
}
```

**Formula breakdown**:
```
If vel = [ω, v] and v = [ω_v, v_v], then:
vel × v = [-ω × ω_v, -ω × v_v - ω_v × v_ω]
```

### Propagation Order

The **forward pass** respects the kinematic tree structure:
```
World (i=0)
  ├─ Body 1
  │  ├─ Body k
  │  └─ ...
  └─ Body j
     └─ ...
```

When processing body i:
1. Parent's cvel is already computed (parent index < i)
2. Body i's cvel is computed by adding parent's contribution
3. This ensures dependencies are satisfied

---

## SECTION 3: mj_rne() - Recursive Newton-Euler Algorithm

### Location
- **File**: `engine_core_smooth.c`
- **Lines**: 2068-2119

### Function Signature
```c
void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result) {
```

### Parameters
- `m` : Model (fixed, contains structure)
- `d` : Data (dynamic, contains states)
- `flg_acc` : Flag; if 1, include inertial term; if 0, compute Coriolis/centrifugal only
- `result` : Output vector of size nv containing joint torques/forces

### Algorithm Overview

RNE computes **M(q)*qacc + C(q,qvel)**, where:
- **M(q)**: Generalized mass matrix (inertia)
- **C(q,qvel)**: Velocity-dependent forces (Coriolis/centrifugal)

The algorithm has three distinct phases:

```
PHASE 1: Forward pass (compute spatial accelerations cacc)
  for body i in 1..nbody-1:
      cacc[i] = cacc[parent[i]] + cdofdot * qvel + cdof * qacc

PHASE 2: Compute spatial forces (using Newton's 2nd law)
  for body i in 1..nbody-1:
      cfrc_body[i] = cinert[i] * cacc[i] + cvel[i] × (cinert[i] * cvel[i])

PHASE 3: Backward pass (project spatial forces to joint torques)
  for body i in nbody-1..1:
      cfrc_body[parent[i]] += cfrc_body[i]
  
  for DOF j in 0..nv-1:
      result[j] = cdof[j] · cfrc_body[body[j]]
```

### Detailed Code Analysis

#### Initialization (Lines 2069-2079)

```c
int nbody = m->nbody, nv = m->nv;
mjtNum tmp[6], tmp1[6];
mj_markStack(d);
mjtNum* loc_cacc = mjSTACKALLOC(d, m->nbody*6, mjtNum);
mjtNum* loc_cfrc_body = mjSTACKALLOC(d, m->nbody*6, mjtNum);

// Set world acceleration to -gravity
mju_zero(loc_cacc, 6);
if (!mjDISABLED(mjDSBL_GRAVITY)) {
    mju_scl3(loc_cacc+3, m->opt.gravity, -1);
}
```

**Key points**:
- Local stack-allocated arrays for intermediate calculations
- World body's acceleration is initialized to **-g** (gravity)
  - This is because the algorithm works in the world frame
  - The negative sign accounts for fictitious forces in non-inertial reference

#### Phase 1: Forward Pass (Lines 2082-2101)

```c
for (int i=1; i < nbody; i++) {
    int bda = m->body_dofadr[i];  // First DOF address
    
    // LINE 2087: cacc = cacc_parent + cdofdot * qvel
    mju_mulDofVec(tmp, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[i]);
    mju_add(loc_cacc+6*i, loc_cacc+6*m->body_parentid[i], tmp, 6);
    
    // LINE 2091-2093: cacc += cdof * qacc (if flag set)
    if (flg_acc) {
        mju_mulDofVec(tmp, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[i]);
        mju_addTo(loc_cacc+6*i, tmp, 6);
    }
    
    // LINE 2097-2100: cfrc_body = cinert * cacc + cvel × (cinert * cvel)
    mju_mulInertVec(loc_cfrc_body+6*i, d->cinert+10*i, loc_cacc+6*i);
    mju_mulInertVec(tmp, d->cinert+10*i, d->cvel+6*i);
    mju_crossForce(tmp1, d->cvel+6*i, tmp);
    mju_addTo(loc_cfrc_body+6*i, tmp1, 6);
}
```

**Mathematical breakdown**:

```
For body i:
  cacc[i] = cacc[parent] + cdof_dot[i] * qvel[i] + cdof[i] * qacc[i]
            └─ inherited  └─ Coriolis      └─ inertial
  
  cfrc_body[i] = I[i] * cacc[i] + cvel[i] × (I[i] * cvel[i])
                 └─ inertia    └─ centrifugal/Coriolis force
```

**Key operation: Coriolis/Centrifugal term**

The term `cvel × (cinert * cvel)` is crucial:
```
Let p = I * cvel  (spatial momentum)
Then cvel × p represents the rate of change of momentum due to rotation

In matrix form: this is equivalent to the Coriolis/centrifugal term in dynamics
```

**Code comment (implied)**: This is why we need both `cvel` and acceleration; the velocity terms create forces even when qacc=0.

#### Phase 2: Clear world forces (Line 2104)

```c
mju_zero(loc_cfrc_body, 6);
```

World body has no forces.

#### Phase 3: Backward Pass - Force Accumulation (Lines 2107-2111)

```c
for (int i=nbody-1; i > 0; i--) {
    if (m->body_parentid[i]) {
        mju_addTo(loc_cfrc_body+6*m->body_parentid[i], loc_cfrc_body+6*i, 6);
    }
}
```

**What happens**:
- Iterate from leaves (body nbody-1) to root
- Accumulate each body's force to its parent
- This implements the **backward pass** of RNE

**Result**: `loc_cfrc_body[parent]` now contains the sum of all forces from the body and its subtree.

#### Phase 4: Project to Joint Space (Lines 2114-2116)

```c
for (int i=0; i < nv; i++) {
    result[i] = mju_dot(d->cdof+6*i, loc_cfrc_body+6*m->dof_bodyid[i], 6);
}
```

**The dot product projection**:
```
τ[i] = cdof[i]^T · cfrc_body[body_of_dof_i]
```

This projects the 6D spatial force to a scalar joint torque/force.

### Key Spatial Algebra Operations Used

#### mju_mulInertVec(res, inert, vec)
**Location**: `engine_util_spatial.c`, lines 434-441

```c
void mju_mulInertVec(mjtNum* restrict res, const mjtNum i[10], const mjtNum v[6]) {
  res[0] = i[0]*v[0] + i[3]*v[1] + i[4]*v[2] - i[8]*v[4] + i[7]*v[5];
  res[1] = i[3]*v[0] + i[1]*v[1] + i[5]*v[2] + i[8]*v[3] - i[6]*v[5];
  res[2] = i[4]*v[0] + i[5]*v[1] + i[2]*v[2] - i[7]*v[3] + i[6]*v[4];
  res[3] = i[8]*v[1] - i[7]*v[2] + i[9]*v[3];
  res[4] = i[6]*v[2] - i[8]*v[0] + i[9]*v[4];
  res[5] = i[7]*v[0] - i[6]*v[1] + i[9]*v[5];
}
```

**What it computes**: Multiplies spatial inertia matrix (10 elements) by 6D vector

The inertia matrix is stored in triangular format:
```
I = [ I_3x3        m*c_cross   ]  = [i[0] i[3] i[4]  i[6] i[7] i[8]]
    [ -m*c_cross   m*I_3x3    ]    [i[3] i[1] i[5]  i[8]  0   i[9]]
                                    [i[4] i[5] i[2]   0   i[9]  0  ]
                                    [i[6] i[8]  0     i[9]  0    0  ]
                                    [i[7]  0   i[9]   0    i[9]  0  ]
                                    [i[8] i[9]  0     0    0   i[9]]

Actually, from code inspection, the ordering is:
i[0:3] = diagonal of inertia
i[3:6] = off-diagonal of inertia
i[6:9] = coupling between rotation and translation
i[9]   = mass
```

#### mju_crossForce(res, vel, f)
**Location**: `engine_util_spatial.c`, lines 385-396

```c
void mju_crossForce(mjtNum res[6], const mjtNum vel[6], const mjtNum f[6]) {
  // Angular part: -ω × τ
  res[0] = -vel[2]*f[1] + vel[1]*f[2];
  res[1] =  vel[2]*f[0] - vel[0]*f[2];
  res[2] = -vel[1]*f[0] + vel[0]*f[1];
  
  // Linear part: -ω × f - ω_v × τ
  res[3] = -vel[2]*f[4] + vel[1]*f[5];
  res[4] =  vel[2]*f[3] - vel[0]*f[5];
  res[5] = -vel[1]*f[3] + vel[0]*f[4];
  
  res[0] += -vel[5]*f[4] + vel[4]*f[5];
  res[1] +=  vel[5]*f[3] - vel[3]*f[5];
  res[2] += -vel[4]*f[3] + vel[3]*f[4];
}
```

**What it computes**: Lie bracket for force vectors: **res = vel × f**

This represents the rate of change of force due to rotation in the velocity frame.

---

## SECTION 4: mj_passive() - Passive Forces

### Location
- **File**: `engine_passive.c`
- **Lines**: 599-688

### Function Signature
```c
void mj_passive(const mjModel* m, mjData* d) {
```

### Algorithm Overview

The function computes and accumulates all passive (velocity-dependent) forces:
1. **Springs**: Restoring forces from displacement
2. **Dampers**: Velocity-proportional forces
3. **Gravity compensation**: Applied via actuators
4. **Fluid forces**: Aerodynamic/hydrodynamic drag
5. **Contact forces**: Passive contact stiffness

### Code Analysis

#### Initialization (Lines 600-612)

```c
int nv = m->nv;

// Clear all passive force vectors
mju_zero(d->qfrc_spring,   nv);
mju_zero(d->qfrc_damper,   nv);
mju_zero(d->qfrc_gravcomp, nv);
mju_zero(d->qfrc_fluid,    nv);
mju_zero(d->qfrc_passive,  nv);

// Both spring and damping disabled: skip all passive forces
if (mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER)) {
    return;
}
```

#### Spring and Damper Forces (Lines 614-615)

Delegates to `mj_springdamper()` function.

### mj_springdamper() - Detailed Analysis

**Location**: `engine_passive.c`, lines 60-460

#### DOF-Level Dampers (Lines 116-123)

```c
if (has_damping) {
    for (int i=0; i < m->nv; i++) {
        mjtNum damping = m->dof_damping[i];
        if (damping != 0) {
            d->qfrc_damper[i] = -damping*d->qvel[i];
        }
    }
}
```

**Formula**: 
```
qfrc_damper[i] = -b[i] * qvel[i]
where b[i] = damping coefficient for DOF i
```

**Physical meaning**: Linear viscous damping
- Force is proportional to velocity
- Negative (opposes motion)
- Applied independently per DOF

#### Joint-Level Springs (Lines 68-113)

```c
if (has_spring) {
    for (int i=0; i < njnt; i++) {
        mjtNum stiffness = m->jnt_stiffness[i];
        if (stiffness == 0) {
            continue;
        }
        
        int padr = m->jnt_qposadr[i];
        int dadr = m->jnt_dofadr[i];
        
        switch ((mjtJoint) m->jnt_type[i]) {
        case mjJNT_FREE:
            // Translational springs (3 DOFs)
            d->qfrc_spring[dadr+0] = -stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
            d->qfrc_spring[dadr+1] = -stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
            d->qfrc_spring[dadr+2] = -stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);
            
            dadr += 3;  // Skip to rotational part
            padr += 3;
            mjFALLTHROUGH;
        
        case mjJNT_BALL:
            // Rotational springs (3 DOFs)
            // For ball joints: convert quaternion difference to angular velocity
            {
                mjtNum dif[3], quat[4];
                mju_copy4(quat, d->qpos+padr);
                mju_normalize4(quat);
                mju_subQuat(dif, quat, m->qpos_spring + padr);
                
                d->qfrc_spring[dadr+0] = -stiffness*dif[0];
                d->qfrc_spring[dadr+1] = -stiffness*dif[1];
                d->qfrc_spring[dadr+2] = -stiffness*dif[2];
            }
            break;
        
        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            // Linear springs (1 DOF)
            d->qfrc_spring[dadr] = -stiffness*(d->qpos[padr] - m->qpos_spring[padr]);
            break;
        }
    }
}
```

**Formulas**:

For hinge/slide joints:
```
τ_spring = -k * (q - q_rest)
where k = spring stiffness
      q = current position
      q_rest = rest position (m->qpos_spring)
```

For ball joints:
```
τ_spring = -k * (q_diff in angular velocity form)
where q_diff is computed by mju_subQuat()
```

**Key detail - mju_subQuat() (engine_util_spatial.c, lines 132-140)**:
```c
void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]) {
    mjtNum qneg[4], qdif[4];
    mju_negQuat(qneg, qb);
    mju_mulQuat(qdif, qneg, qa);      // qdif = qb^{-1} * qa
    mju_quat2Vel(res, qdif, 1);       // Convert to angular velocity (3D)
}
```

This converts the quaternion difference (representing orientation error) into a 3D angular velocity, which is then scaled by stiffness to get torque.

#### Tendon-Level Spring-Dampers (Lines 420-459)

```c
for (int i=0; i < ntendon; i++) {
    mjtNum stiffness = m->tendon_stiffness[i] * has_spring;
    mjtNum damping = m->tendon_damping[i] * has_damping;
    
    // Disabled: skip
    if (stiffness == 0 && damping == 0) {
        continue;
    }
    
    // Compute spring force
    mjtNum length = d->ten_length[i];
    mjtNum lower = m->tendon_lengthspring[2*i];
    mjtNum upper = m->tendon_lengthspring[2*i+1];
    mjtNum frc_spring = 0;
    if (length > upper) {
        frc_spring = stiffness * (upper - length);  // Stretched beyond max
    } else if (length < lower) {
        frc_spring = stiffness * (lower - length);  // Compressed below min
    }
    
    // Compute damper force
    mjtNum frc_damper = -damping * d->ten_velocity[i];
    
    // Project to joint space and accumulate
    if (issparse) {
        if (frc_spring || frc_damper) {
            int end = d->ten_J_rowadr[i] + d->ten_J_rownnz[i];
            for (int j=d->ten_J_rowadr[i]; j < end; j++) {
                int k = d->ten_J_colind[j];
                mjtNum J = d->ten_J[j];
                d->qfrc_spring[k] += J * frc_spring;
                d->qfrc_damper[k] += J * frc_damper;
            }
        }
    } else {
        if (frc_spring) mju_addToScl(d->qfrc_spring, d->ten_J+i*nv, frc_spring, nv);
        if (frc_damper) mju_addToScl(d->qfrc_damper, d->ten_J+i*nv, frc_damper, nv);
    }
}
```

**Formula**:
```
frc_spring = k * max(0, upper - length) + k * min(0, lower - length)
           = k * clip(length, lower, upper) - k * length  (simplified)
frc_damper = -b * tendon_velocity[i]

qfrc[j] += J[j, i] * frc_spring + J[j, i] * frc_damper
```

### Passive Force Accumulation (Lines 626-627)

```c
mju_add(d->qfrc_passive, d->qfrc_spring, d->qfrc_damper, nv);
```

This combines spring and damper forces into the unified `qfrc_passive` vector.

---

## SECTION 5: Key Differences from Simplified Implementations

### 5.1 Composite Frame vs. Global Frame

**Simplified implementation**: Works in global frame with all positions/velocities expressed globally.

**MuJoCo implementation**: Uses **composite frames** (body-local, centered at each body's COM).

**Advantages**:
- Automatic handling of kinematic constraints (only 6 DOF per body, not 12)
- More efficient numerically (smaller matrices)
- Natural representation of rotational dynamics

**Data structures**:
```
Global frame: xpos[3], xmat[3x3], xquat[4]
Composite frame: xipos[3], ximat[3x3] (for inertial COM)
                 cvel[6], cacc[6], cfrc_body[6]
```

### 5.2 Motion Subspace as Basis Vectors

**Simplified implementation**: Joint constraints handled separately as equality constraints.

**MuJoCo implementation**: Motion subspace (cdof) represents allowed directions of motion.

**cdof structure**:
```
For each DOF j:
  cdof[6*j:6*j+6] = spatial basis vector
  
This encodes: "how does this DOF contribute to spatial velocity?"
```

**Why it matters**:
- Efficiently represents kinematic structure
- Automatic composition across multiple joints
- Natural Jacobian representation

### 5.3 Coriolis/Centrifugal Forces via Lie Bracket

**Simplified implementation**: Explicitly compute Coriolis matrix elements.

**MuJoCo implementation**: Uses spatial cross-products (Lie brackets).

**Formula**:
```
Velocity-dependent acceleration = cvel × cdof
(spatial cross-product of velocity with motion subspace)

Centrifugal force = cvel × (I * cvel)
(spatial cross-product of velocity with spatial momentum)
```

**Why it's more robust**:
- Automatically consistent with spatial algebra
- Handles all joint types uniformly
- Numerically stable

### 5.4 Spatial Inertia Matrix

**Standard approach**: 3x3 rotational inertia I, mass m separate.

**MuJoCo approach**: Unified 6x6 spatial inertia I_spatial.

**Storage (10 elements)**:
```
I_spatial = [ I_rot          m * c_cross_skew ]
            [-m * c_cross_skew    m * I_3×3   ]

where c = position of COM relative to reference frame
```

**Benefits**:
- Single multiplication for both rotation and translation
- Automatic coupling between rotational/translational dynamics
- Correct centrifugal force computation

### 5.5 Numerical Optimizations

#### Quaternion normalization in spring computation
```c
mju_copy4(quat, d->qpos+padr);
mju_normalize4(quat);
mju_subQuat(dif, quat, m->qpos_spring + padr);
```
This normalizes before computing difference to avoid accumulated rounding errors.

#### Using parent velocity for cdofdot
```c
// Comment: "using the old velocity may be more accurate numerically"
mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));
```
Uses parent's velocity instead of just-updated body velocity because:
```
cvel × cdof = Coriolis term
cvel_new × cdof = same (since cdof doesn't depend on cvel)
cvel_old × cdof = numerically better (avoids round-trip computation)
```

#### Stack allocation for temporary arrays
```c
mj_markStack(d);
mjtNum* loc_cacc = mjSTACKALLOC(d, m->nbody*6, mjtNum);
```
Uses stack allocator instead of malloc for:
- Automatic deallocation
- Better cache locality
- No fragmentation

---

## SECTION 6: Complete Mathematical Derivation

### 6.1 Spatial Velocity Propagation

**Forward kinematics** (from mj_comVel):

```
For body i with parent p:
  cvel[i] = cvel[p] + Σ_j (cdof[ij] * qvel[j])
  
where cdof[ij] is the j-th motion subspace of body i
```

### 6.2 Spatial Acceleration

**From Newton's second law** (forward pass of RNE):

```
cacc[i] = cacc[p] + Σ_j (cdof_dot[ij] * qvel[j]) + Σ_j (cdof[ij] * qacc[j])
```

where:
```
cdof_dot[ij] = cvel[p] × cdof[ij]  (Lie bracket, Coriolis term)
```

### 6.3 Spatial Force Computation

**Newton's second law for rigid body**:

```
cfrc_body[i] = I[i] * cacc[i] + cvel[i] × (I[i] * cvel[i])
             = inertial force + centrifugal force
```

The centrifugal term:
```
cvel[i] × (I[i] * cvel[i])

= [ω, v] × (I_rot*ω + m*c_cross*v, -m*c_cross_T*ω + m*v)
= gravity-like terms + Coriolis terms
```

### 6.4 Recursive Force Accumulation

**Backward pass**:

```
For i from nbody-1 down to 1:
  cfrc_body[parent[i]] += cfrc_body[i]
```

Result: `cfrc_body[i]` contains total force on body i AND all descendants.

### 6.5 Projection to Joint Space

**Final projection**:

```
For DOF j on body i:
  τ[j] = cdof[j]^T · cfrc_body[i]
       = ⟨cdof[j], cfrc_body[i]⟩
```

This inner product projects the 6D spatial force onto the 1D motion subspace.

### 6.6 Final Equation of Motion

**After RNE (inverse dynamics)**:

```
τ = M(q) * qacc + C(q, qvel)
```

where:
- `τ` = result vector from RNE with flg_acc=1
- `M(q) * qacc` = inertial term (proportional to qacc)
- `C(q, qvel)` = velocity-dependent term (from cdof_dot and cvel × forces)

---

## SECTION 7: Code-to-Formula Correspondence

### Cross-Product Formulas Used

**Standard 3D cross product**:
```c
void mju_cross(mjtNum res[3], const mjtNum a[3], const mjtNum b[3]) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}
// Formula: a × b
```

**Motion cross-product** (spatial Lie bracket):
```c
void mju_crossMotion(mjtNum res[6], const mjtNum vel[6], const mjtNum v[6]) {
    // vel = [ω, v], v = [ω_v, v_v]
    // res = [-ω × ω_v, -ω × v_v - ω_v × v_ω]
    // Represents: [ω, v] × [ω_v, v_v]
}
// Formula: vel × v (Lie bracket for motion)
```

**Force cross-product**:
```c
void mju_crossForce(mjtNum res[6], const mjtNum vel[6], const mjtNum f[6]) {
    // vel = [ω, v_f], f = [τ, f_f]
    // res = [-ω × τ - v_f × f_f, -ω × f_f]
    // Represents: [ω, v_f] × [τ, f_f]
}
// Formula: vel × f (Lie bracket for force)
```

**Quaternion rotation**:
```c
void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4]) {
    mjtNum tmp[3] = {
        quat[0]*vec[0] + quat[2]*vec[2] - quat[3]*vec[1],
        quat[0]*vec[1] + quat[3]*vec[0] - quat[1]*vec[2],
        quat[0]*vec[2] + quat[1]*vec[1] - quat[2]*vec[0]
    };
    res[0] = vec[0] + 2 * (quat[2]*tmp[2] - quat[3]*tmp[1]);
    res[1] = vec[1] + 2 * (quat[3]*tmp[0] - quat[1]*tmp[2]);
    res[2] = vec[2] + 2 * (quat[1]*tmp[1] - quat[2]*tmp[0]);
}
// Formula: q * v * q^{-1} (quaternion rotation)
```

---

## SECTION 8: Common Pitfalls and How MuJoCo Avoids Them

### Pitfall 1: Inconsistent Coordinate Frames

**Problem**: Mixing global and local coordinates causes errors.

**MuJoCo solution**: 
- Consistently uses composite frames (local to each body's COM)
- Transformations via spatial rotation matrices and translations
- All computations in single, well-defined frame per body

### Pitfall 2: Incorrect Coriolis Terms

**Problem**: Using velocity-dependent accelerations incorrectly leads to unstable dynamics.

**MuJoCo solution**:
- Uses Lie brackets: `cvel × cdof` for cdof_dot
- Centrifugal force via: `cvel × (I * cvel)`
- Automatically consistent with spatial algebra theory

### Pitfall 3: Singular Configurations

**Problem**: Gimbal lock with Euler angles; quaternions help but need care.

**MuJoCo solution**:
- Uses quaternions throughout (never Euler angles for states)
- Special handling for ball joint springs (converts via mju_subQuat)
- Normalizes quaternions frequently to avoid drift

### Pitfall 4: Joint Type Inconsistency

**Problem**: Different joint types (hinge, ball, slide) need different Coriolis handling.

**MuJoCo solution**:
- Explicit switch statements for each joint type in mj_comVel
- Ball joints: 3 Coriolis terms (one per axis)
- Hinge/Slide: 1 Coriolis term per DOF
- Free joints: 0 Coriolis terms for translation, 3 for rotation
- All unified under spatial algebra framework

### Pitfall 5: Numerical Precision

**Problem**: Accumulated rounding errors in iterative calculations.

**MuJoCo solution**:
- Uses parent's velocity (not updated) for cdof_dot computation
- Normalizes quaternions explicitly
- Stack allocation for temporary arrays (better cache, less fragmentation)
- Double-precision floating point (mjtNum = double by default)

---

## SECTION 9: Performance Characteristics

### Time Complexity

**mj_comVel()**: O(nv)
- Single forward pass
- Each DOF processed once

**mj_rne()**: O(nv) 
- Forward pass: O(nbody) ≤ O(nv)
- Backward pass: O(nbody)
- Projection: O(nv)
- Total: O(nv) for tree-structured systems

**mj_passive()**: O(njnt + ntendon)
- Linear in number of joints and tendons
- Not dependent on state dimension

### Memory Usage

**mj_rne()** allocates:
- `loc_cacc`: 6*nbody doubles = 48*nbody bytes
- `loc_cfrc_body`: 6*nbody doubles = 48*nbody bytes
- Total: O(nbody) temporary memory

This is stack-allocated and automatically freed.

---

## SECTION 10: Integration with Forward Dynamics

### Call Sequence in mj_forward()

```
1. mj_kinematics()      - Compute body positions/orientations
2. mj_comPos()          - Map inertias and DOF to COM frame
3. mj_comVel()          - Compute spatial velocities (cvel, cdof_dot)
4. mj_rne()             - Compute forces (with flg_acc=0 for Coriolis)
5. mj_rnePostConstraint() - Complete acceleration computation
6. Integration step     - Update velocities and positions
```

### Data Dependencies

```
mj_comVel depends on:
  - d->qvel (joint velocities)
  - d->cdof (motion subspaces, precomputed in mj_comPos)
  - d->cvel (previous frame's velocities, but this is a forward pass)

mj_rne depends on:
  - d->cdof (motion subspaces)
  - d->cdof_dot (from mj_comVel)
  - d->cvel (from mj_comVel)
  - d->cinert (spatial inertias, precomputed in mj_comPos)
  - d->qacc (desired accelerations)
  - d->qvel (current velocities)
  - m->body_parentid (tree structure)
  - m->dof_bodyid (DOF-to-body mapping)

mj_passive depends on:
  - d->qpos (current positions)
  - d->qvel (current velocities)
  - m->dof_damping, jnt_stiffness (parameters)
  - m->qpos_spring (rest positions)
```

---

## SECTION 11: Verification and Testing

### Key Numerical Properties to Check

1. **Energy conservation**: With no damping, energy should be conserved
   ```
   E = 0.5 * qvel^T * M * qvel + potential_energy
   dE/dt ≈ 0 (within numerical error)
   ```

2. **Force balance at equilibrium**:
   ```
   With qacc=0, C+gravity ≈ 0 for configuration at equilibrium
   ```

3. **Coriolis symmetry**:
   ```
   Centrifugal forces should create accelerations perpendicular to velocity
   ```

4. **Damping dissipation**:
   ```
   With damping only (no springs), energy should monotonically decrease
   dE/dt < 0 always
   ```

### Debugging Techniques

**Print spatial vectors**:
```c
// [ω_x, ω_y, ω_z, v_x, v_y, v_z]
printf("cvel[%d] = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
       i, d->cvel[6*i], d->cvel[6*i+1], ..., d->cvel[6*i+5]);
```

**Check motion subspace**:
```c
printf("cdof[%d] = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
       dof, d->cdof[6*dof], ..., d->cdof[6*dof+5]);
```

**Verify force projection**:
```c
mjtNum check = mju_dot(d->cdof+6*i, loc_cfrc_body+6*body, 6);
printf("Projected force DOF %d: %.6f\n", i, check);
```

---

## APPENDIX A: Data Layout Reference

### d->cvel[6*nbody] - Spatial Velocities
```
d->cvel[0:6]     = world body (always [0,0,0,0,0,0])
d->cvel[6:12]    = body 1: [ω_1x, ω_1y, ω_1z, v_1x, v_1y, v_1z]
d->cvel[12:18]   = body 2: [ω_2x, ω_2y, ω_2z, v_2x, v_2y, v_2z]
...
```

### d->cdof[6*nv] - Motion Subspace
```
d->cdof[0:6]     = DOF 0: [ω_0x, ω_0y, ω_0z, v_0x, v_0y, v_0z]
d->cdof[6:12]    = DOF 1: [ω_1x, ω_1y, ω_1z, v_1x, v_1y, v_1z]
...
```

For a hinge joint:
```
cdof = [axis_x, axis_y, axis_z, (axis × offset)_x, (axis × offset)_y, (axis × offset)_z]
```

For a slide joint:
```
cdof = [0, 0, 0, axis_x, axis_y, axis_z]
```

### d->cinert[10*nbody] - Spatial Inertia
```
d->cinert[10*i + 0:3] = I_xx, I_yy, I_zz (rotational inertia diagonal)
d->cinert[10*i + 3:6] = I_xy, I_xz, I_yz (rotational inertia off-diagonal)
d->cinert[10*i + 6:9] = m*c_x, m*c_y, m*c_z (mass × COM offset)
d->cinert[10*i + 9]   = m (mass)
```

---

## APPENDIX B: RNE Algorithm Summary

### Input
- `m->body_parentid[]`: Parent body index for each body
- `m->dof_bodyid[]`: Body index for each DOF
- `d->cdof[6*nv]`: Motion subspace vectors
- `d->cdof_dot[6*nv]`: Time derivatives of motion subspaces
- `d->cvel[6*nbody]`: Spatial velocities (from mj_comVel)
- `d->cinert[10*nbody]`: Spatial inertia matrices
- `d->qvel[nv]`: Joint velocities
- `d->qacc[nv]`: Joint accelerations (if flg_acc=1)
- `opt.gravity`: Gravity vector

### Output
- `result[nv]`: Joint torques/forces

### Algorithm

```
1. Initialize: loc_cacc[0] = [0, 0, 0, -g_x, -g_y, -g_z]

2. Forward pass (i = 1 to nbody-1):
     loc_cacc[i] = loc_cacc[parent[i]] + cdof_dot * qvel + (flg_acc ? cdof * qacc : 0)
     loc_cfrc_body[i] = I[i] * loc_cacc[i] + cvel[i] × (I[i] * cvel[i])

3. Backward pass (i = nbody-1 down to 1):
     loc_cfrc_body[parent[i]] += loc_cfrc_body[i]

4. Projection (j = 0 to nv-1):
     result[j] = cdof[j] · loc_cfrc_body[body[j]]
```

---

## APPENDIX C: Key Code Snippets

### Complete mj_comVel (Simplified)
```c
void mj_comVel(const mjModel* m, mjData* d) {
  mju_zero(d->cvel, 6);  // World velocity = 0
  
  for (int i=1; i < m->nbody; i++) {
    mjtNum cvel[6];
    int bda = m->body_dofadr[i];
    
    // Copy parent velocity
    mju_copy(cvel, d->cvel+6*m->body_parentid[i], 6);
    
    // Add contribution from each DOF
    for (int j=0; j < m->body_dofnum[i]; j++) {
      mjtNum tmp[6];
      
      // Compute cdof_dot (Coriolis term)
      if (is_rotational_joint) {
        mju_crossMotion(d->cdof_dot+6*(bda+j), cvel, d->cdof+6*(bda+j));
      } else {
        mju_zero(d->cdof_dot+6*(bda+j), 6);
      }
      
      // Update velocity
      mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
      mju_addTo(cvel, tmp, 6);
    }
    
    mju_copy(d->cvel+6*i, cvel, 6);
  }
}
```

### Spatial Cross-Product Usage
```c
// In mj_comVel: compute Coriolis term
mju_crossMotion(cdofdot, cvel, cdof);
// Result: cdofdot = cvel × cdof

// In mj_rne: compute centrifugal force
mju_mulInertVec(tmp, cinert, cvel);         // tmp = I * cvel
mju_crossForce(centrifugal, cvel, tmp);     // centrifugal = cvel × (I*cvel)
```

### Damping Force Computation
```c
// Linear viscous damping
for (int i=0; i < nv; i++) {
  mjtNum b = dof_damping[i];
  qfrc_damper[i] = -b * qvel[i];
}

// Spring force (hinge joint)
for (int i=0; i < njnt; i++) {
  mjtNum k = jnt_stiffness[i];
  int padr = jnt_qposadr[i];
  int dadr = jnt_dofadr[i];
  qfrc_spring[dadr] = -k * (qpos[padr] - qpos_spring[padr]);
}
```

---

## Conclusion

MuJoCo's dynamics implementation is highly optimized using:
1. **Spatial algebra** for unified treatment of rotational and translational dynamics
2. **Recursive algorithms** (RNE) with O(nv) complexity
3. **Composite frames** for numerical efficiency and natural constraint handling
4. **Sophisticated spatial cross-products** for automatic Coriolis/centrifugal computation
5. **Careful numerical handling** (quaternion normalization, smart velocity ordering)

The implementation is production-quality with rigorous adherence to spatial algebra theory while maintaining practical computational efficiency.

