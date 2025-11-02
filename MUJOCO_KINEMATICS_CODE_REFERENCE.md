# MuJoCo Forward Kinematics - Exact Code Reference

## Complete Code Snippets with Line Numbers

### 1. Main Kinematics Function

**File**: `mujoco/src/engine/engine_core_smooth.c`
**Lines**: 37-177

```c
void mj_kinematics(const mjModel* m, mjData* d) {
  int nbody = m->nbody, nsite = m->nsite, ngeom = m->ngeom;

  // Initialize world body
  mju_zero3(d->xpos);
  mju_unit4(d->xquat);
  mju_zero3(d->xipos);
  mju_zero(d->xmat, 9);
  mju_zero(d->ximat, 9);
  d->xmat[0] = d->xmat[4] = d->xmat[8] = 1;
  d->ximat[0] = d->ximat[4] = d->ximat[8] = 1;

  // Main loop: compute global position/orientation for all bodies
  for (int i=1; i < nbody; i++) {
    mjtNum xpos[3], xquat[4];
    int jntadr = m->body_jntadr[i];
    int jntnum = m->body_jntnum[i];

    // Free joint: directly from qpos
    if (jntnum == 1 && m->jnt_type[jntadr] == mjJNT_FREE) {
      int qadr = m->jnt_qposadr[jntadr];
      mju_copy3(xpos, d->qpos+qadr);
      mju_copy4(xquat, d->qpos+qadr+3);
      mju_normalize4(xquat);  // <-- NORMALIZATION #1
      mju_copy3(d->xanchor+3*jntadr, xpos);
      mju_copy3(d->xaxis+3*jntadr, m->jnt_axis+3*jntadr);
    }
    // Regular or no joint
    else {
      int pid = m->body_parentid[i];
      
      // Get body position and orientation (from model or mocap)
      mjtNum *bodypos, *bodyquat, quat[4];
      if (m->body_mocapid[i] >= 0) {
        bodypos = d->mocap_pos + 3*m->body_mocapid[i];
        mju_copy4(quat, d->mocap_quat + 4*m->body_mocapid[i]);
        mju_normalize4(quat);  // <-- NORMALIZATION #2
        bodyquat = quat;
      } else {
        bodypos = m->body_pos+3*i;
        bodyquat = m->body_quat+4*i;
      }

      // Apply fixed translation and rotation relative to parent
      if (pid) {
        mju_mulMatVec3(xpos, d->xmat+9*pid, bodypos);
        mju_addTo3(xpos, d->xpos+3*pid);
        mju_mulQuat(xquat, d->xquat+4*pid, bodyquat);
      } else {
        // Parent is the world
        mju_copy3(xpos, bodypos);
        mju_copy4(xquat, bodyquat);
      }

      // Accumulate joints: THIS IS THE HINGE JOINT PROCESSING
      mjtNum xanchor[3], xaxis[3];
      for (int j=0; j < jntnum; j++) {
        int jid = jntadr + j;
        int qadr = m->jnt_qposadr[jid];
        mjtJoint jtype = m->jnt_type[jid];

        // Compute axis in global frame
        mju_rotVecQuat(xaxis, m->jnt_axis+3*jid, xquat);

        // Compute anchor in global frame
        mju_rotVecQuat(xanchor, m->jnt_pos+3*jid, xquat);
        mju_addTo3(xanchor, xpos);

        // Apply joint transformation
        switch (jtype) {
        case mjJNT_SLIDE:
          mju_addToScl3(xpos, xaxis, d->qpos[qadr] - m->qpos0[qadr]);
          break;

        case mjJNT_BALL:
        case mjJNT_HINGE:
          {
            // Convert joint angle to local quaternion
            mjtNum qloc[4];
            if (jtype == mjJNT_BALL) {
              mju_copy4(qloc, d->qpos+qadr);
              mju_normalize4(qloc);  // <-- NORMALIZATION #3
            } else {
              // THIS IS THE KEY: axis-angle to quaternion for hinge
              mju_axisAngle2Quat(qloc, m->jnt_axis+3*jid, d->qpos[qadr] - m->qpos0[qadr]);
            }

            // Apply rotation to body's quaternion
            mju_mulQuat(xquat, xquat, qloc);

            // ============================================================
            // CRITICAL: Correct for off-center rotation (LINES 133-137)
            // ============================================================
            mjtNum vec[3];
            mju_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);
            mju_sub3(xpos, xanchor, vec);
            // ============================================================
          }
          break;

        default:
          mjERROR("unknown joint type %d", jtype);
        }

        // Store joint anchor and axis for later use
        mju_copy3(d->xanchor+3*jid, xanchor);
        mju_copy3(d->xaxis+3*jid, xaxis);
      }
    }

    // Normalize quaternion to prevent drift
    mju_normalize4(xquat);  // <-- NORMALIZATION #4
    mju_copy4(d->xquat+4*i, xquat);
    mju_copy3(d->xpos+3*i, xpos);
    mju_quat2Mat(d->xmat+9*i, xquat);
  }

  // Compute inertial frame transforms
  for (int i=1; i < nbody; i++) {
    mj_local2Global(d, d->xipos+3*i, d->ximat+9*i,
                    m->body_ipos+3*i, m->body_iquat+4*i,
                    i, m->body_sameframe[i]);
  }

  // Compute geom transforms
  for (int i=0; i < ngeom; i++) {
    mj_local2Global(d, d->geom_xpos+3*i, d->geom_xmat+9*i,
                    m->geom_pos+3*i, m->geom_quat+4*i,
                    m->geom_bodyid[i], m->geom_sameframe[i]);
  }

  // Compute site transforms
  for (int i=0; i < nsite; i++) {
    mj_local2Global(d, d->site_xpos+3*i, d->site_xmat+9*i,
                    m->site_pos+3*i, m->site_quat+4*i,
                    m->site_bodyid[i], m->site_sameframe[i]);
  }
}
```

---

### 2. Quaternion Axis-Angle Conversion

**File**: `mujoco/src/engine/engine_util_spatial.c`
**Lines**: 95-112

```c
// Convert axis-angle to quaternion
void mju_axisAngle2Quat(mjtNum res[4], const mjtNum axis[3], mjtNum angle) {
  // Zero angle: null quat
  if (angle == 0) {
    res[0] = 1;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;
  }

  // Regular processing
  else {
    mjtNum s = mju_sin(angle*0.5);
    res[0] = mju_cos(angle*0.5);
    res[1] = axis[0]*s;
    res[2] = axis[1]*s;
    res[3] = axis[2]*s;
  }
}
```

**Why stable**:
- Zero angle check avoids sin/cos rounding errors
- Formula guarantees unit quaternion: sin²(a/2) + cos²(a/2) = 1
- No normalization step needed

---

### 3. Quaternion Vector Rotation

**File**: `mujoco/src/engine/engine_util_spatial.c`
**Lines**: 27-52

```c
// Rotate vector by quaternion: v' = R(q) * v
void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4]) {
  // Zero vec: zero res
  if (vec[0] == 0 && vec[1] == 0 && vec[2] == 0) {
    mju_zero3(res);
  }

  // Null quat (identity): copy vec
  else if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    mju_copy3(res, vec);
  }

  // Regular processing
  else {
    // tmp = q_w * v + cross(q_xyz, v)
    mjtNum tmp[3] = {
      quat[0]*vec[0] + quat[2]*vec[2] - quat[3]*vec[1],
      quat[0]*vec[1] + quat[3]*vec[0] - quat[1]*vec[2],
      quat[0]*vec[2] + quat[1]*vec[1] - quat[2]*vec[0]
    };

    // res = v + 2 * cross(q_xyz, tmp)
    res[0] = vec[0] + 2 * (quat[2]*tmp[2] - quat[3]*tmp[1]);
    res[1] = vec[1] + 2 * (quat[3]*tmp[0] - quat[1]*tmp[2]);
    res[2] = vec[2] + 2 * (quat[1]*tmp[1] - quat[2]*tmp[0]);
  }
}
```

**Formula used**: v' = v + 2*q_w*cross(q_xyz, v) + 2*cross(q_xyz, cross(q_xyz, v))

**Why stable**:
- Identity check avoids error accumulation for static bodies
- Fast formula with minimal intermediate values
- No matrix conversion needed

---

### 4. Quaternion Multiplication

**File**: `mujoco/src/engine/engine_util_spatial.c`
**Lines**: 65-76

```c
// Multiply quaternions: res = qa * qb (Hamilton product)
void mju_mulQuat(mjtNum res[4], const mjtNum qa[4], const mjtNum qb[4]) {
  mjtNum tmp[4] = {
    qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3],
    qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2],
    qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1],
    qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
  res[3] = tmp[3];
}
```

**Why stable**:
- Standard Hamilton product formula
- Uses temp to avoid overwriting input
- Each component computed once (no cancellation issues)

---

### 5. Quaternion Normalization

**File**: `mujoco/src/engine/engine_util_blas.c`
**Lines**: 248-265

```c
// Normalize quaternion to unit length
mjtNum mju_normalize4(mjtNum vec[4]) {
  mjtNum norm = mju_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + 
                         vec[2]*vec[2] + vec[3]*vec[3]);

  if (norm < mjMINVAL) {  // mjMINVAL = 1e-8
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
    vec[3] = 0;
  } else if (mju_abs(norm - 1) > mjMINVAL) {
    // Only normalize if deviation > threshold (avoid unnecessary work)
    mjtNum normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
    vec[3] *= normInv;
  }

  return norm;
}
```

**Stability tricks**:
1. Threshold check (mjMINVAL = 1e-8): avoids normalizing already-unit quaternions
2. Reciprocal computation: 1/norm computed once, multiplied by each component
3. Zero vector handling: resets to identity instead of dividing by zero

---

### 6. Quaternion to Matrix Conversion

**File**: `mujoco/src/engine/engine_util_spatial.c`
**Lines**: 144-182

```c
// Convert quaternion to 3D rotation matrix
void mju_quat2Mat(mjtNum res[9], const mjtNum quat[4]) {
  // Null quat: identity matrix
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    res[0] = 1; res[1] = 0; res[2] = 0;
    res[3] = 0; res[4] = 1; res[5] = 0;
    res[6] = 0; res[7] = 0; res[8] = 1;
  }

  // Regular processing
  else {
    // Precompute all products to minimize operations
    const mjtNum q00 = quat[0]*quat[0];
    const mjtNum q01 = quat[0]*quat[1];
    const mjtNum q02 = quat[0]*quat[2];
    const mjtNum q03 = quat[0]*quat[3];
    const mjtNum q11 = quat[1]*quat[1];
    const mjtNum q12 = quat[1]*quat[2];
    const mjtNum q13 = quat[1]*quat[3];
    const mjtNum q22 = quat[2]*quat[2];
    const mjtNum q23 = quat[2]*quat[3];
    const mjtNum q33 = quat[3]*quat[3];

    // Diagonal elements
    res[0] = q00 + q11 - q22 - q33;
    res[4] = q00 - q11 + q22 - q33;
    res[8] = q00 - q11 - q22 + q33;

    // Off-diagonal elements
    res[1] = 2*(q12 - q03);
    res[2] = 2*(q13 + q02);
    res[3] = 2*(q12 + q03);
    res[5] = 2*(q23 - q01);
    res[6] = 2*(q13 - q02);
    res[7] = 2*(q23 + q01);
  }
}
```

**Why stable**:
1. Identity check avoids expensive computation for static bodies
2. Precomputed products minimize repeated multiplications
3. Formula: M = (q0²-q1²-q2²-q3²)I + 2[q0[q1,q2,q3]×] + 2[q1,q2,q3]T[q1,q2,q3]

---

### 7. Subtree Center of Mass Computation

**File**: `mujoco/src/engine/engine_core_smooth.c`
**Lines**: 181-259 (within mj_comPos)

```c
void mj_comPos(const mjModel* m, mjData* d) {
  int nbody = m->nbody, njnt = m->njnt;

  // Step 1: Initialize with body moment (mass * position)
  for (int i=0; i < nbody; i++) {
    mju_scl3(d->subtree_com+3*i, d->xipos+3*i, m->body_mass[i]);
  }

  // Step 2: Accumulate to parent in BACKWARD pass
  // (children processed before parents)
  for (int i=nbody-1; i > 0; i--) {  // <-- Goes from high to low!
    int j = m->body_parentid[i];
    mju_addTo3(d->subtree_com+3*j, d->subtree_com+3*i);
  }

  // Step 3: Normalize by subtree mass to get COM
  for (int i=0; i < nbody; i++) {
    if (m->body_subtreemass[i] < mjMINVAL) {
      mju_copy3(d->subtree_com+3*i, d->xipos+3*i);
    } else {
      mju_scl3(d->subtree_com+3*i, d->subtree_com+3*i, 1.0/m->body_subtreemass[i]);
    }
  }

  // ... (inertia mapping follows) ...
```

**Why stable**:
1. Backward pass ensures children processed before parents
2. Uses precomputed subtree_mass values
3. Checks for zero mass (safety)

---

### 8. Inertia Transformation with Parallel Axis Theorem

**File**: `mujoco/src/engine/engine_util_spatial.c`
**Lines**: 400-430

```c
// Transform inertia to center-of-mass frame with parallel axis theorem
void mju_inertCom(mjtNum res[10], const mjtNum inert[3], const mjtNum mat[9],
                  const mjtNum dif[3], mjtNum mass) {
  // Step 1: Rotate diagonal inertia to global frame
  // tmp = diag(inert) * mat'
  mjtNum tmp[9] = {
    mat[0]*inert[0], mat[3]*inert[0], mat[6]*inert[0],
    mat[1]*inert[1], mat[4]*inert[1], mat[7]*inert[1],
    mat[2]*inert[2], mat[5]*inert[2], mat[8]*inert[2]
  };

  // Step 2: Rotate to global: res_rot = mat * diag(inert) * mat'
  res[0] = mat[0]*tmp[0] + mat[1]*tmp[3] + mat[2]*tmp[6];  // I_xx
  res[1] = mat[3]*tmp[1] + mat[4]*tmp[4] + mat[5]*tmp[7];  // I_yy
  res[2] = mat[6]*tmp[2] + mat[7]*tmp[5] + mat[8]*tmp[8];  // I_zz
  res[3] = mat[0]*tmp[1] + mat[1]*tmp[4] + mat[2]*tmp[7];  // I_xy
  res[4] = mat[0]*tmp[2] + mat[1]*tmp[5] + mat[2]*tmp[8];  // I_xz
  res[5] = mat[3]*tmp[2] + mat[4]*tmp[5] + mat[5]*tmp[8];  // I_yz

  // Step 3: Apply parallel axis theorem
  // I_com = I_body - m*[dif]^2 where [dif]^2 is skew-symmetric square
  res[0] += mass*(dif[1]*dif[1] + dif[2]*dif[2]);
  res[1] += mass*(dif[0]*dif[0] + dif[2]*dif[2]);
  res[2] += mass*(dif[0]*dif[0] + dif[1]*dif[1]);
  res[3] -= mass*dif[0]*dif[1];
  res[4] -= mass*dif[0]*dif[2];
  res[5] -= mass*dif[1]*dif[2];

  // Step 4: Store translation component (mass*offset)
  res[6] = mass*dif[0];
  res[7] = mass*dif[1];
  res[8] = mass*dif[2];

  // Step 5: Store mass
  res[9] = mass;
}
```

**10-element format**: [I_xx, I_yy, I_zz, I_xy, I_xz, I_yz, m*dx, m*dy, m*dz, mass]

**Why stable**:
1. Diagonal inertia (3 values) before rotation, not full matrix
2. Parallel axis theorem applied directly
3. Precomputed matrix products minimize operations

---

### 9. Motion Subspace for Hinge in COM Frame

**File**: `mujoco/src/engine/engine_util_spatial.c`
**Lines**: 445-457

```c
// Express motion axis in center-of-mass frame
void mju_dofCom(mjtNum res[6], const mjtNum axis[3], const mjtNum offset[3]) {
  // HINGE case: offset is non-NULL
  if (offset) {
    mju_copy3(res, axis);              // res[0:3] = angular velocity = axis
    mju_cross(res+3, axis, offset);    // res[3:6] = linear velocity = axis x offset
  }
  // SLIDE case: offset is NULL
  else {
    mju_zero3(res);                    // res[0:3] = 0 (no rotation)
    mju_copy3(res+3, axis);            // res[3:6] = linear velocity = axis
  }
}
```

**For HINGE**: cdof = [ω, v] = [axis, axis × offset]

This is the 6D spatial motion subspace vector.

---

### 10. Spatial Velocity and cdof_dot Computation

**File**: `mujoco/src/engine/engine_core_smooth.c`
**Lines**: 1931-1997

```c
// Compute center-of-mass velocity and cdof time derivatives
void mj_comVel(const mjModel* m, mjData* d) {
  int nbody = m->nbody;

  // World body has zero velocity
  mju_zero(d->cvel, 6);

  // Forward pass over bodies
  for (int i=1; i < nbody; i++) {
    int bda = m->body_dofadr[i];
    int dofnum = m->body_dofnum[i];

    // Get parent's velocity
    mjtNum cvel[6];
    mju_copy(cvel, d->cvel+6*m->body_parentid[i], 6);

    // Process each DOF on this body
    mjtNum cdofdot[36];
    for (int j=0; j < dofnum; j++) {
      mjtNum tmp[6];

      // Determine joint type
      switch ((mjtJoint) m->jnt_type[m->dof_jntid[bda+j]]) {
      case mjJNT_FREE:
        // cdofdot = 0 for translational components
        mju_zero(cdofdot, 18);

        // Update velocity
        mju_mulDofVec(tmp, d->cdof+6*bda, d->qvel+bda, 3);
        mju_addTo(cvel, tmp, 6);
        j += 3;
        mjFALLTHROUGH;

      case mjJNT_BALL:
        // Compute all 3 cdofdots using parent velocity
        for (int k=0; k < 3; k++) {
          mju_crossMotion(cdofdot+6*(j+k), cvel, d->cdof+6*(bda+j+k));
        }

        // Update velocity
        mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 3);
        mju_addTo(cvel, tmp, 6);
        j += 2;
        break;

      default:  // HINGE and SLIDE joints
        // cdof_dot = cvel x cdof (spatial cross product)
        // Using OLD cvel (before adding current DOF's velocity)
        // This is more numerically accurate because crossMotion(cdof, cdof) = 0
        mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));

        // Update velocity with this DOF's contribution
        mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
        mju_addTo(cvel, tmp, 6);
      }
    }

    // Store results
    mju_copy(d->cvel+6*i, cvel, 6);
    mju_copy(d->cdof_dot+6*bda, cdofdot, 6*dofnum);
  }
}
```

**Key insight** (line 1982-1984):
Uses OLD velocity to compute cdof_dot because crossMotion(cdof, cdof) = 0 always.

---

### 11. Local to Global Transform

**File**: `mujoco/src/engine/engine_core_util.c`
**Lines**: 853-893

```c
// Transform from body-local to global frame
void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9],
                     const mjtNum pos[3], const mjtNum quat[4],
                     int body, mjtByte sameframe) {
  mjtSameFrame sf = sameframe;

  // Transform position
  if (xpos && pos) {
    switch (sf) {
    case mjSAMEFRAME_NONE:
    case mjSAMEFRAME_BODYROT:
    case mjSAMEFRAME_INERTIAROT:
      // Rotate then translate
      mju_mulMatVec3(xpos, d->xmat+9*body, pos);
      mju_addTo3(xpos, d->xpos+3*body);
      break;
    case mjSAMEFRAME_BODY:
      // Use body position directly
      mju_copy3(xpos, d->xpos+3*body);
      break;
    case mjSAMEFRAME_INERTIA:
      // Use inertial position directly
      mju_copy3(xpos, d->xipos+3*body);
      break;
    }
  }

  // Transform orientation
  if (xmat && quat) {
    mjtNum tmp[4];
    switch (sf) {
    case mjSAMEFRAME_NONE:
      // Apply quaternion
      mju_mulQuat(tmp, d->xquat+4*body, quat);
      mju_quat2Mat(xmat, tmp);
      break;
    case mjSAMEFRAME_BODY:
    case mjSAMEFRAME_BODYROT:
      // Use body orientation
      mju_copy(xmat, d->xmat+9*body, 9);
      break;
    case mjSAMEFRAME_INERTIA:
    case mjSAMEFRAME_INERTIAROT:
      // Use inertial orientation
      mju_copy(xmat, d->ximat+9*body, 9);
      break;
    }
  }
}
```

---

## Summary of Stability Mechanisms by Line

| Mechanism | Location | Lines | Purpose |
|-----------|----------|-------|---------|
| Free joint normalization | mj_kinematics | 63, 79 | Normalize raw qpos input |
| Zero angle check | axisAngle2Quat | 97-102 | Avoid trig errors at rest |
| Off-center rotation fix | mj_kinematics | 133-137 | Maintain joint anchor point |
| Body quaternion normalization | mj_kinematics | 151 | Prevent drift after multiplications |
| Identity quaternion check | rotVecQuat | 34-35 | Optimize static bodies |
| Recursive vector rotation | rotVecQuat | 40-50 | Fast rotation formula |
| Quaternion product | mulQuat | 66-75 | Efficient 16-op formula |
| Threshold-based renorm | normalize4 | 256 | Only renormalize when needed |
| Zero vector handling | normalize4 | 251-255 | Safe normalization |
| Backward COM accumulation | comPos | 190 | Correct dependency order |
| Diagonal inertia storage | inertCom | 403-405 | Minimize operations |
| Parallel axis theorem | inertCom | 416-421 | Direct application |
| Old velocity for cdofdot | comVel | 1982-1985 | Numerically stable |

---

## Key Invariants

1. **Quaternion norm**: Always maintained at 1.0 (up to mjMINVAL threshold)
2. **Joint anchor point**: Always stays at xanchor computed from parent frame
3. **Parent-to-child**: Each body depends only on immediate parent
4. **Backward accumulation**: COM computed with correct dependency order
5. **Inertia frame**: Always in global frame centered at subtree COM

