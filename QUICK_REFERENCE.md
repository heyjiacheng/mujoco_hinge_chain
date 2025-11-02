# MuJoCo Dynamics - Quick Reference Guide

## File Locations

| Function | File | Lines |
|----------|------|-------|
| mj_comVel() | engine_core_smooth.c | 1932-1997 |
| mj_rne() | engine_core_smooth.c | 2068-2119 |
| mj_passive() | engine_passive.c | 599-688 |
| mju_crossMotion() | engine_util_spatial.c | 370-381 |
| mju_crossForce() | engine_util_spatial.c | 385-396 |
| mju_mulInertVec() | engine_util_spatial.c | 434-441 |

## Spatial Vector Format

```
v[6] = [ω_x, ω_y, ω_z, v_x, v_y, v_z]
       [rotation/angular | translation/linear]
```

## Key Formulas

### 1. Velocity Propagation
```
cvel[i] = cvel[parent] + Σ(cdof[j] * qvel[j])
```

### 2. Coriolis Term
```
cdof_dot[j] = cvel[parent] × cdof[j]   (spatial cross-product)
```

### 3. Spatial Acceleration
```
cacc[i] = cacc[parent] + cdof_dot * qvel + (flg_acc ? cdof * qacc : 0)
```

### 4. Spatial Force
```
cfrc[i] = I[i] * cacc[i] + cvel[i] × (I[i] * cvel[i])
        = inertial_force + centrifugal_force
```

### 5. Joint Torque/Force
```
τ[j] = cdof[j]^T · cfrc_body[body[j]]   (dot product projection)
```

### 6. Damping
```
qfrc_damper[i] = -b[i] * qvel[i]
```

### 7. Spring (Hinge/Slide)
```
qfrc_spring[j] = -k[j] * (q[j] - q_rest[j])
```

## Algorithm Flowchart

### mj_comVel() Flow
```
Initialize: cvel[world] = 0

For each body i:
  1. cvel = copy(cvel[parent[i]])
  
  For each DOF j on body i:
    2. If rotational: cdof_dot[j] = cvel × cdof[j]
    3. Else: cdof_dot[j] = 0
    4. cvel += cdof[j] * qvel[j]
  
  Store: cvel[i] and cdof_dot[*]
```

### mj_rne() Flow
```
Phase 1 - Forward Pass:
  cacc[world] = -gravity
  
  For each body i:
    cacc[i] = cacc[parent] + cdof_dot * qvel
    if (flg_acc) cacc[i] += cdof * qacc
    cfrc[i] = I[i] * cacc[i] + cvel[i] × (I[i] * cvel[i])

Phase 2 - Backward Pass:
  For each body i (leaf to root):
    cfrc[parent] += cfrc[i]

Phase 3 - Projection:
  For each DOF j:
    result[j] = cdof[j] · cfrc[body[j]]
```

### mj_passive() Flow
```
Initialize: qfrc_spring = 0, qfrc_damper = 0

For each DOF i:
  qfrc_damper[i] = -dof_damping[i] * qvel[i]

For each joint j:
  qfrc_spring[dof[j]] = -k[j] * (q[j] - q_rest[j])
  (special handling for ball joints via quaternion)

For each tendon t:
  frc_spring = k * clip(length, lower, upper)
  frc_damper = -b * tendon_velocity
  Accumulate to qfrc_spring/damper via Jacobian

Final: qfrc_passive = qfrc_spring + qfrc_damper
```

## Spatial Cross-Products

### Motion Cross-Product
```
mju_crossMotion(res, vel, v):
  res = vel × v
  res[0:3] = -ω × ω_v
  res[3:6] = -ω × v_v - ω_v × v_ω
```

### Force Cross-Product
```
mju_crossForce(res, vel, f):
  res = vel × f
  res[0:3] = -ω × τ - v_f × f_f
  res[3:6] = -ω × f_f
```

## Data Structures Cheat Sheet

### d->cvel[6*nbody] - Spatial Velocities
```
[body_i]:
  [0:3] = ω (angular velocity)
  [3:6] = v (linear velocity at COM)
```

### d->cdof[6*nv] - Motion Subspace
```
For DOF j on body with COM offset from joint:
  HINGE: [axis, axis × offset]
  SLIDE: [0, 0, 0, axis]
  BALL:  [axis_1, axis_2, axis_3] (3 DOFs)
  FREE:  [I_3x3, 0; 0, I_3x3] (6 DOFs)
```

### d->cinert[10*nbody] - Spatial Inertia
```
Compact representation of 6×6 matrix:
  [0:3]   = I_diag (diagonal of rotational inertia)
  [3:6]   = I_off  (off-diagonal of rotational inertia)
  [6:9]   = m*c    (mass × COM offset)
  [9]     = m      (mass)
```

### d->cfrc_body[6*nbody] - Spatial Forces
```
[body_i]:
  [0:3] = τ (torque/moment)
  [3:6] = f (force)
```

## Critical Code Sections

### Coriolis Computation (mj_comVel)
```c
// Line 1970 (BALL joint)
mju_crossMotion(cdofdot+6*(j+k), cvel, d->cdof+6*(bda+j+k));

// Line 1985 (HINGE/SLIDE)
mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));
```

### Centrifugal Force (mj_rne)
```c
// Lines 2097-2100
mju_mulInertVec(loc_cfrc_body+6*i, d->cinert+10*i, loc_cacc+6*i);
mju_mulInertVec(tmp, d->cinert+10*i, d->cvel+6*i);
mju_crossForce(tmp1, d->cvel+6*i, tmp);
mju_addTo(loc_cfrc_body+6*i, tmp1, 6);
```

### Damping (mj_passive)
```c
// Lines 116-123
for (int i=0; i < m->nv; i++) {
  mjtNum damping = m->dof_damping[i];
  if (damping != 0) {
    d->qfrc_damper[i] = -damping*d->qvel[i];
  }
}
```

## Common Operations

### Get body's DOF range
```c
int bda = m->body_dofadr[i];      // First DOF address
int dofnum = m->body_dofnum[i];   // Number of DOFs
```

### Access spatial vectors
```c
mjtNum* vel = d->cvel + 6*i;      // Body i's velocity
mjtNum* dof = d->cdof + 6*j;      // DOF j's motion subspace
```

### Check joint type
```c
switch ((mjtJoint) m->jnt_type[jid]) {
  case mjJNT_FREE:   // 6 DOF
  case mjJNT_BALL:   // 3 DOF (rotation)
  case mjJNT_HINGE:  // 1 DOF (rotation)
  case mjJNT_SLIDE:  // 1 DOF (translation)
}
```

## Debugging Tips

### Print spatial velocity
```c
printf("cvel[%d] = [%.6f, %.6f, %.6f | %.6f, %.6f, %.6f]\n",
       i, d->cvel[6*i], d->cvel[6*i+1], d->cvel[6*i+2],
       d->cvel[6*i+3], d->cvel[6*i+4], d->cvel[6*i+5]);
```

### Print motion subspace
```c
printf("cdof[%d] = [%.6f, %.6f, %.6f | %.6f, %.6f, %.6f]\n",
       j, d->cdof[6*j], d->cdof[6*j+1], d->cdof[6*j+2],
       d->cdof[6*j+3], d->cdof[6*j+4], d->cdof[6*j+5]);
```

### Verify spatial force projection
```c
mjtNum tau = mju_dot(d->cdof+6*j, d->cfrc_body+6*body, 6);
printf("Projected torque DOF %d: %.6f\n", j, tau);
```

## Performance Summary

| Operation | Complexity | Notes |
|-----------|------------|-------|
| mj_comVel() | O(nv) | Single forward pass |
| mj_rne() | O(nv) | Tree-structured recursion |
| mj_passive() | O(njnt) | Linear in joints |
| mju_crossMotion() | O(1) | 6 arithmetic ops |
| mju_crossForce() | O(1) | 6 arithmetic ops |
| mju_mulInertVec() | O(1) | ~36 arithmetic ops |

## Key Equations

```
Position Kinematics:
  x[i] = x[parent] + R[parent] @ p_rel
  q[i] = q[parent] * q_rel

Spatial Velocity:
  cvel[i] = cvel[parent] + Σ(cdof[j] * qvel[j])

Coriolis Effect:
  cdof_dot = cvel × cdof
  
Spatial Acceleration:
  cacc[i] = cacc[parent] + cdof_dot*qvel + cdof*qacc

Spatial Dynamics:
  cfrc = I*cacc + cvel×(I*cvel)

Joint Equation:
  τ = M(q)*qacc + C(q,qvel) + τ_passive
```

## Numerical Stability Notes

1. **Quaternion normalization**: Done before spring computation (line 96, engine_passive.c)
2. **Parent velocity for cdof_dot**: Used instead of updated velocity for stability
3. **Stack allocation**: Better cache locality and automatic cleanup
4. **Double precision**: mjtNum = double (except when compiled otherwise)

## Integration Order

```
mj_kinematics()      ← positions/orientations
  ↓
mj_comPos()          ← inertias, motion subspace
  ↓
mj_comVel()          ← velocities, Coriolis
  ↓
mj_rne()             ← forces/torques
  ↓
mj_passive()         ← springs, dampers
  ↓
Integration step     ← update state
```

---

**Generated**: November 2, 2025
**Tool**: Claude Code (Haiku 4.5)
**Completeness**: All core dynamics functions documented with exact line numbers
