# MuJoCo Forward Kinematics Implementation for Hinge Joints

## Executive Summary

MuJoCo's forward kinematics for hinge joints is stable due to a carefully designed sequence of operations that:
1. **Propagates rotations incrementally** through parent-to-child transforms using quaternions
2. **Corrects for off-center rotation** to maintain the joint anchor point invariant
3. **Normalizes quaternions** at strategic points to prevent drift
4. **Uses fast quaternion math** with special-case optimizations

The stability comes from these key design choices:
- Quaternion representation avoids gimbal lock
- Normalization prevents numerical drift
- Off-center rotation correction (lines 133-137) ensures geometric consistency
- Parent-to-child propagation (lines 87-95) maintains transformation correctness

---

## 1. mj_kinematics() - Complete Forward Kinematics

**Location**: `/mujoco/src/engine/engine_core_smooth.c`, lines 37-177

### 1.1 Initialization (Lines 40-47)

Sets world body (body 0) to identity:
- Position = [0, 0, 0]
- Quaternion = [1, 0, 0, 0] (identity)
- Rotation matrix = Identity

**Why**: Establishes the world as the reference frame for all transformations.

### 1.2 Hinge Joint Processing (Lines 118-138)

The critical implementation:

```
Step 1: Convert angle to rotation quaternion
  mju_axisAngle2Quat(qloc, axis, angle)
  Returns: q = [cos(angle/2), sin(angle/2)*axis]

Step 2: Apply rotation to body quaternion
  mju_mulQuat(xquat, xquat, qloc)
  Result: xquat represents cumulative rotation

Step 3: CORRECT FOR OFF-CENTER ROTATION
  vec = rotVecQuat(jnt_pos, xquat)
  xpos = xanchor - vec
  
  This keeps the joint anchor point fixed!
```

**Key insight**: Without the correction, the body would rotate around the wrong center.

### 1.3 Finalize Body Transform (Lines 150-154)

```
mju_normalize4(xquat)     // Prevent drift
mju_copy4(d->xquat+4*i, xquat)
mju_copy3(d->xpos+3*i, xpos)
mju_quat2Mat(d->xmat+9*i, xquat)
```

---

## 2. mj_comPos() - Composite Inertia and Motion Subspace

**Location**: `/mujoco/src/engine/engine_core_smooth.c`, lines 181-259

### 2.1 Subtree Center of Mass (Lines 184-202)

Three-step process:
1. **Initialize**: subtree_com[i] = mass[i] * xipos[i]
2. **Accumulate backward**: For each body, add to parent's COM
3. **Normalize**: Divide by total subtree mass

Result: COM of body and all descendants

### 2.2 Composite Inertia (Lines 207-213)

```
For each body:
  offset = body_pos - subtree_com
  Apply parallel axis theorem:
    I_final = I_rotated + m*(offset_cross)^2
```

Uses `mju_inertCom()` which:
1. Rotates inertia tensor to global frame
2. Applies parallel axis theorem
3. Stores mass*offset components

### 2.3 Motion Subspace for Hinge (Line 254-256)

```
mju_dofCom(cdof, xaxis, offset)
  Returns: [axis, axis x offset]
  
This is the 6D motion subspace in COM frame.
```

---

## 3. Numerical Stability Mechanisms

### 3.1 Quaternion Normalization

**When**: After each body's kinematics, after free joints, after mocap

**How**: `mju_normalize4()` with threshold check
```
norm = sqrt(q[0]^2 + q[1]^2 + q[2]^2 + q[3]^2)
if norm < mjMINVAL:  // mjMINVAL = 1e-8
  reset to identity
else if |norm - 1| > mjMINVAL:
  normalize
```

**Why**: Prevents quaternion drift from accumulated multiplication errors.

### 3.2 Zero Angle Optimization

In `mju_axisAngle2Quat()`:
```
if (angle == 0)
  return [1, 0, 0, 0]  // Identity, no sin/cos needed
```

**Why**: Avoids rounding errors for zero rotations (common at rest).

### 3.3 Identity Check Optimization

In `mju_rotVecQuat()` and `mju_quat2Mat()`:
```
if (quat == [1, 0, 0, 0])
  return simplified result
```

**Why**: Avoids error accumulation for non-rotating bodies.

### 3.4 Off-Center Rotation Correction

**Lines 133-137** - THE KEY STABILITY TRICK:

```c
// Before rotation: xanchor is the joint anchor in global frame
// After rotation: jnt_pos rotates to a new position

// Where does the local joint position end up after rotation?
mju_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);

// Move body so anchor stays fixed
mju_sub3(xpos, xanchor, vec);
```

**Why this matters**:
- Without this: Body center rotates around a changing point
- With this: Joint anchor stays geometrically fixed
- Ensures kinematic consistency: joint constraints are automatically satisfied

### 3.5 Diagonal Inertia Storage

In `mju_inertCom()`:
```
Store only 3 diagonal values, not 9-element matrix
This reduces:
  - Memory usage
  - Floating point operations  
  - Rounding error accumulation
```

### 3.6 Backward Accumulation for COM

Processes children BEFORE parents:
```c
for (int i=nbody-1; i > 0; i--) {  // High to low
    subtree_com[parent] += subtree_com[child]
}
```

**Why**: Ensures correct order of accumulation.

---

## 4. Mathematical Details for Hinge Joints

### 4.1 Axis-Angle to Quaternion

`mju_axisAngle2Quat(res, axis, angle)`:

```
If angle == 0:
  q = [1, 0, 0, 0]  (identity)

Else:
  s = sin(angle/2)
  q = [cos(angle/2), axis[0]*s, axis[1]*s, axis[2]*s]
  
Guarantees: |q| = 1 (unit quaternion, no normalization needed!)
```

### 4.2 Quaternion Multiplication

`mju_mulQuat(res, qa, qb)`:

Hamilton product using 4 multiplications and additions:
```
q_w = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3]
q_x = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2]
q_y = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1]
q_z = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0]
```

### 4.3 Quaternion Vector Rotation

`mju_rotVecQuat(res, vec, quat)`:

```
tmp = [q_w*v_x + q_y*v_z - q_z*v_y,
       q_w*v_y + q_z*v_x - q_x*v_z,
       q_w*v_z + q_x*v_y - q_y*v_x]

res = v + 2 * (q_y*tmp_z - q_z*tmp_y,
               q_z*tmp_x - q_x*tmp_z,
               q_x*tmp_y - q_y*tmp_x)
```

Uses formula: v' = v + 2*q_w*cross(q_xyz, v) + 2*cross(q_xyz, cross(q_xyz, v))

### 4.4 Quaternion to Matrix

`mju_quat2Mat(res, quat)`:

For non-identity quaternions, computes:
```
M[0,0] = q_w^2 + q_x^2 - q_y^2 - q_z^2
M[1,1] = q_w^2 - q_x^2 + q_y^2 - q_z^2
M[2,2] = q_w^2 - q_x^2 - q_y^2 + q_z^2
M[0,1] = 2*(q_x*q_y - q_w*q_z)
... (6 off-diagonal elements)
```

Uses precomputed products to minimize operations.

### 4.5 Motion Subspace for Hinge

`mju_dofCom(res, axis, offset)`:

With offset (HINGE case):
```
res[0:3] = axis           // Angular velocity part
res[3:6] = axis × offset  // Linear velocity part = w × r
```

This is a 6D spatial motion vector [ω; v] in COM frame.

---

## 5. Complete Data Flow for a Hinge Chain

### Position Phase: mj_kinematics()

```
Body 0 (World):
  xpos = [0, 0, 0]
  xquat = [1, 0, 0, 0]
  xmat = I

Body 1:
  Get parent transform (body 0 = identity)
  Apply body's fixed offset
  Joint 0 (hinge at angle θ₁):
    qloc = [cos(θ₁/2), axis₁*sin(θ₁/2)]
    xquat = xquat * qloc
    Correct position for rotation
    Store xanchor, xaxis
  Normalize xquat
  Compute xmat from xquat

Body 2:
  Get parent transform (body 1)
  Apply body's fixed offset
  Joint 1 (hinge at angle θ₂):
    qloc = [cos(θ₂/2), axis₂*sin(θ₂/2)]
    xquat = xquat * qloc
    Correct position for rotation
    Store xanchor, xaxis
  Normalize xquat
  Compute xmat from xquat

... (repeat for remaining bodies)
```

### Position Phase: mj_comPos()

```
Compute subtree_com:
  Backward pass from leaves to root
  Normalize by subtree mass

Compute cinert:
  For each body, apply parallel axis theorem
  Maps inertia to COM frame

Compute cdof:
  For each hinge: cdof = [axis, axis × offset_to_com]
```

### Velocity Phase: mj_comVel()

```
For each body in forward order:
  cvel_i = cvel_parent
  For each DOF on body:
    cdof_dot = cvel_parent × cdof
    cvel_i += cdof * qvel
  Store cvel_i, cdof_dot
```

---

## 6. Why Stability Is Guaranteed

### Reason 1: Quaternion Normalization Prevents Drift

After n quaternion multiplications:
- Without normalization: norm can drift to 0.999... or 1.001...
- With normalization: norm is forced back to 1.0 every body
- Drift is reset to zero, not allowed to accumulate

### Reason 2: Off-Center Correction Enforces Constraints

The kinematic constraint is: joint anchor must be at xanchor.

Without correction:
- Body rotates around body_pos (WRONG - constraint violated)
- Position error grows

With correction:
- Body rotates around joint anchor (CORRECT)
- Position is adjusted to keep anchor fixed
- Constraint is automatically maintained

### Reason 3: Parent-to-Child Propagation

Errors don't compound through the tree:
- Each body depends on parent's transform (1 level back)
- Position error in parent affects child additively, not multiplicatively
- Linear error propagation, not exponential

### Reason 4: Zero Checks Avoid Introduction of Errors

When angle = 0:
- Direct identity quaternion, no sin/cos rounding
- When quat = identity: simplified operations
- No unnecessary computations = no unnecessary rounding

### Reason 5: Inertia Stored Minimally

- Diagonal form (3 values) instead of full matrix (9 values)
- Fewer multiplications = fewer rounding errors
- Parallel axis theorem applied directly, not via matrix operations

---

## 7. Comparison with Alternative Approaches

### Why Not Use Matrices?

Matrices for chain rotations:
```
M_total = M_1 * M_2 * M_3 * ... * M_n
```

Problems:
1. Each matrix multiply: 27 multiplications
2. Accumulates rounding errors n times
3. Must explicitly renormalize to keep orthogonal
4. Gimbal lock in some parameterizations

Quaternions instead:
```
q_total = q_1 ⊗ q_2 ⊗ q_3 ⊗ ... ⊗ q_n
```

Benefits:
1. Each multiply: 16 multiplications (25% fewer)
2. Normalization is just L2 normalization
3. No gimbal lock
4. Can interpolate smoothly (Slerp)

### Why Not Use Euler Angles?

Euler angles (e.g., ZYX convention):
```
θ = [roll, pitch, yaw]
```

Problems:
1. Gimbal lock when middle angle = ±90°
2. Non-linear mapping to rotations
3. Difficulty with constraint satisfaction
4. Singularities at certain configurations

Quaternions avoid all these issues.

---

## 8. Key Code Locations Reference

| Component | File | Lines | Stability Trick |
|-----------|------|-------|-----------------|
| Main kinematics loop | engine_core_smooth.c | 50-155 | Parent-to-child propagation |
| Off-center correction | engine_core_smooth.c | 133-137 | Maintains joint anchor |
| Quaternion normalization | engine_core_smooth.c | 151 | Prevents drift |
| Axis-angle conversion | engine_util_spatial.c | 95-112 | Zero-angle optimization |
| Quaternion multiplication | engine_util_spatial.c | 65-76 | Efficient Hamilton product |
| Vector rotation | engine_util_spatial.c | 27-52 | Fast formula, identity checks |
| Quaternion to matrix | engine_util_spatial.c | 144-182 | Precomputed products |
| Normalization | engine_util_blas.c | 248-265 | Threshold-based (avoid unnecessary work) |
| COM computation | engine_core_smooth.c | 184-202 | Backward accumulation |
| Inertia transform | engine_util_spatial.c | 400-430 | Diagonal form, parallel axis theorem |

---

## 9. Summary: The Complete Stability Picture

MuJoCo's forward kinematics achieves stability through:

1. **Representation**: Quaternions (no gimbal lock, compact, smooth)
2. **Propagation**: Parent-to-child (linear error growth)
3. **Correction**: Off-center fix (constraint enforcement)
4. **Normalization**: Periodic (prevents drift accumulation)
5. **Optimization**: Zero/identity checks (avoids unnecessary computation)
6. **Compactness**: Diagonal inertia (fewer operations = fewer errors)

The **off-center rotation correction** (lines 133-137) is the key innovation that distinguishes MuJoCo from naive kinematics. By ensuring the joint anchor stays fixed, it automatically satisfies kinematic constraints without explicit constraint checking.

This allows fast, stable, and accurate forward kinematics suitable for real-time physics simulation.
