# MuJoCo Forward Kinematics Analysis - Complete Index

## Document Overview

This directory contains a complete, line-by-line analysis of how MuJoCo implements stable forward kinematics for hinge joints.

### Main Analysis Documents

1. **MUJOCO_KINEMATICS_STABILITY.md** (12 KB, 438 lines)
   - **Purpose**: Executive summary and conceptual overview
   - **Contents**:
     - Why MuJoCo kinematics is stable (6 reasons)
     - How quaternions prevent gimbal lock
     - Off-center rotation correction (the KEY insight)
     - Normalization strategies
     - Comparison with alternative approaches (matrices, Euler angles)
   - **Best for**: Understanding WHY the implementation works
   - **Read this first** for overall understanding

2. **MUJOCO_KINEMATICS_CODE_REFERENCE.md** (18 KB, 628 lines)
   - **Purpose**: Exact code snippets with annotations
   - **Contents**:
     - Complete source code for 11 key functions
     - Line numbers from original MuJoCo source
     - Inline comments explaining each operation
     - Stability tricks highlighted
     - Summary table of all mechanisms
   - **Best for**: Understanding HOW each function works
   - **Reference this** when implementing or debugging

### Supporting Documents (Pre-existing)

3. **MUJOCO_CODE_DETAILS.md** (23 KB, 782 lines)
   - Deep dive into MuJoCo source structure
   - File organization and dependencies

4. **MUJOCO_DYNAMICS_ANALYSIS.md** (12 KB, 452 lines)
   - Forward dynamics pipeline
   - Constraint solving

---

## Quick Reference: Key Code Locations

### Forward Kinematics Main Loop
- **File**: `mujoco/src/engine/engine_core_smooth.c`
- **Lines**: 37-177 (mj_kinematics function)
- **What it does**: Computes global position/orientation for all bodies

### The Critical Off-Center Rotation Correction
- **File**: `mujoco/src/engine/engine_core_smooth.c`
- **Lines**: 133-137
- **What it does**: Keeps joint anchor fixed during rotation
- **Why important**: Maintains kinematic constraints

### Axis-Angle to Quaternion Conversion (for Hinge)
- **File**: `mujoco/src/engine/engine_util_spatial.c`
- **Lines**: 95-112 (mju_axisAngle2Quat)
- **What it does**: Converts hinge angle to quaternion
- **Why stable**: Zero-angle check, formula guarantees unit quaternion

### Quaternion Normalization
- **File**: `mujoco/src/engine/engine_util_blas.c`
- **Lines**: 248-265 (mju_normalize4)
- **What it does**: Keeps quaternion at unit length
- **Why important**: Prevents drift accumulation

### Composite Inertia Computation
- **File**: `mujoco/src/engine/engine_core_smooth.c`
- **Lines**: 181-259 (mj_comPos)
- **What it does**: Prepares dynamics data (cinert, cdof, cdof_dot)

### Spatial Velocity Computation
- **File**: `mujoco/src/engine/engine_core_smooth.c`
- **Lines**: 1932-1997 (mj_comVel)
- **What it does**: Computes body velocities and cdof_dot

---

## The Four Normalizations

MuJoCo normalizes quaternions at these key points:

1. **Free Joint Input** (line 63)
   - Normalizes raw qpos values from user
   - Ensures integration errors don't compound

2. **Mocap Bodies** (line 79)
   - Normalizes external tracking data
   - Safety check for user-provided rotations

3. **Ball Joint** (line 125)
   - Normalizes input quaternion from qpos
   - Ensures unit quaternion before multiplication

4. **Body Finalization** (line 151)
   - Normalizes after all joint transformations
   - Resets drift accumulated during joint processing

---

## The Off-Center Correction Explained

### Problem
After rotating a body around a joint, the body center moves if the joint is not at the body center.

```
Before rotation:          After rotation (WRONG):
    [anchor]                  [joint anchor]
       |  (joint)                  |
       |                           |
    [body]                      [body] <- MOVED!
```

### Solution (Lines 133-137)
```c
// Where did the local joint position end up?
mju_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);

// Move body so anchor stays at original position
mju_sub3(xpos, xanchor, vec);
```

Result: Joint anchor stays fixed, body rotates around it correctly.

---

## Numerical Stability Checklist

MuJoCo achieves stability through:

- [x] **Quaternion representation**: Avoids gimbal lock
- [x] **Periodic normalization**: Prevents drift (every body)
- [x] **Off-center correction**: Maintains kinematic constraints
- [x] **Zero-angle optimization**: Avoids unnecessary sin/cos
- [x] **Identity checks**: Avoids error accumulation for static bodies
- [x] **Threshold-based renormalization**: Only renormalizes when needed
- [x] **Backward accumulation**: Correct dependency order for COM
- [x] **Diagonal inertia storage**: Minimizes floating-point operations
- [x] **Fast quaternion formulas**: Minimizes intermediate values
- [x] **Parent-to-child propagation**: Linear error growth

---

## Data Flow Summary

### Phase 1: mj_kinematics() (Position)

```
Initialize world body (identity transform)
  ↓
For each body (1 to nbody):
  Get parent's transform
  Apply body's fixed offset
  For each joint:
    Compute joint axis/anchor in global frame
    HINGE: Convert angle to quaternion
    Apply rotation to body frame
    Correct for off-center rotation ← KEY
  Normalize quaternion ← Prevent drift
  Convert quaternion to matrix
  Store: xpos, xquat, xmat
  ↓
Transform inertial frames, geoms, sites
```

### Phase 2: mj_comPos() (Prepare Dynamics)

```
Compute subtree center of mass (backward pass)
  ↓
Map inertias to COM frame (parallel axis theorem)
  ↓
Compute motion subspace cdof in COM frame
```

### Phase 3: mj_comVel() (Velocity)

```
For each body (1 to nbody):
  Get parent's velocity
  For each DOF:
    Compute cdof_dot = cvel × cdof
    Update velocity
  Store: cvel, cdof_dot
```

---

## Implementation Notes for Your Code

### When Implementing Hinge Joints

1. **Always use quaternions**, not matrices or Euler angles
   - More efficient (16 ops vs 27 for matrix multiply)
   - Avoid gimbal lock
   - Enable smooth interpolation

2. **Normalize after every quaternion multiply**
   - But use threshold check (only if deviation > 1e-8)
   - Avoids unnecessary work

3. **Correct for off-center rotation**
   - This is the non-obvious part
   - Keep anchor point fixed during rotation
   - See lines 133-137 for exact formula

4. **Use axis-angle conversion directly**
   - Don't go through matrices
   - Guarantees unit quaternion (sin²+cos²=1)

5. **Check for zero angles**
   - Avoid sin/cos computation at rest
   - Reduces rounding errors

---

## Validation Questions

To verify your implementation is stable:

1. **Quaternion norm**: Check d->xquat[i] has norm = 1.0 (±1e-8)
2. **Joint anchor**: Verify d->xanchor doesn't drift
3. **Long simulations**: Run 1000+ steps without position errors
4. **Large rotations**: Test ±360 degree rotations
5. **Chain stability**: Verify kinematic chain doesn't violate constraints
6. **Numerical test**: Compare with MuJoCo reference implementation

---

## References in Code

### Key Functions Used
- `mju_axisAngle2Quat()`: Angle to quaternion
- `mju_mulQuat()`: Quaternion multiplication
- `mju_rotVecQuat()`: Vector rotation by quaternion
- `mju_quat2Mat()`: Quaternion to matrix
- `mju_normalize4()`: Quaternion normalization
- `mju_dofCom()`: Motion subspace in COM frame
- `mju_inertCom()`: Inertia transformation
- `mj_local2Global()`: Local to global transform

### Constants
- `mjMINVAL`: 1e-8 (numerical threshold)
- `mjPI`: 3.14159...

---

## File Organization

```
mujoco/src/engine/
├── engine_core_smooth.c
│   ├── mj_kinematics() [lines 37-177]
│   ├── mj_comPos() [lines 181-259]
│   └── mj_comVel() [lines 1932-1997]
├── engine_util_spatial.c
│   ├── mju_rotVecQuat() [lines 27-52]
│   ├── mju_mulQuat() [lines 65-76]
│   ├── mju_axisAngle2Quat() [lines 95-112]
│   ├── mju_quat2Mat() [lines 144-182]
│   ├── mju_inertCom() [lines 400-430]
│   └── mju_dofCom() [lines 445-457]
├── engine_util_blas.c
│   └── mju_normalize4() [lines 248-265]
└── engine_core_util.c
    └── mj_local2Global() [lines 853-893]
```

---

## Further Reading

### In This Repository
- MUJOCO_KINEMATICS_STABILITY.md - Conceptual overview
- MUJOCO_KINEMATICS_CODE_REFERENCE.md - Exact code snippets

### External Resources
- MuJoCo GitHub: https://github.com/deepmind/mujoco
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- Quaternion Math: https://en.wikipedia.org/wiki/Quaternion

---

## Questions This Analysis Answers

1. **Why is MuJoCo so stable?**
   - Quaternions + periodic normalization + off-center correction

2. **How does it handle hinge rotations?**
   - Converts angle to quaternion, applies rotation, corrects position

3. **Why normalize quaternions?**
   - Prevents drift from accumulated multiplication errors

4. **What's the off-center correction?**
   - Keeps joint anchor fixed while body rotates around it

5. **How are dynamics computed from kinematics?**
   - mj_comPos() transforms kinematics into COM-based frame for dynamics

6. **Why use quaternions instead of matrices?**
   - More efficient, avoid gimbal lock, enable interpolation

7. **How often are computations done?**
   - Kinematics: Every mj_forward() call
   - COM/dynamics prep: Every mj_forward() call
   - Velocity: Every mj_forward() call

---

## Summary

MuJoCo's forward kinematics is stable because:

1. It uses **quaternions** to represent rotations (no gimbal lock)
2. It **normalizes periodically** to prevent drift
3. It **corrects position** to keep joint anchors fixed
4. It uses **efficient, numerically-friendly** formulas
5. It **checks for special cases** (zero angles, identity quaternions)
6. It **minimizes floating-point operations** (diagonal inertia, fast formulas)

The implementation is elegantly designed to be both fast and accurate.

