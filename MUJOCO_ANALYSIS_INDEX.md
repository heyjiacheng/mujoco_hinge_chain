# MuJoCo Dynamics Analysis - Complete Documentation Index

## Overview

This directory contains an extremely thorough analysis of how MuJoCo implements dynamics computation, with complete code walkthrough, mathematical derivations, and practical reference guides.

## Document Guide

### 1. QUICK_REFERENCE.md (6.8 KB)
**START HERE** - Quick lookup guide for the most important information.

Contains:
- Function locations (file + line numbers)
- Spatial vector format
- Key formulas (velocity, acceleration, forces)
- Algorithm flowcharts
- Data structure cheat sheet
- Critical code sections
- Debugging tips
- Performance summary

**Best for**: Quick lookups, understanding the big picture at a glance

---

### 2. DYNAMICS_ANALYSIS_SUMMARY.txt (14 KB)
Structured summary with exact line numbers and detailed breakdowns.

Contains:
- Key findings (3 core functions analyzed)
- Critical spatial algebra operations
- Numerical optimizations
- Exact line-by-line analysis:
  - mj_comVel() (Lines 1932-1997)
  - mj_rne() (Lines 2068-2119)
  - Spring/Damper (Lines 60-460)
- Mathematical formulas implemented
- Key code snippets
- Data structure reference
- Performance analysis
- Critical insights
- Common implementation mistakes

**Best for**: Understanding exactly what each line does, specific implementation details

---

### 3. MUJOCO_DYNAMICS_DETAILED.md (34 KB)
Complete technical analysis with extensive commentary.

11 Sections covering:
1. Spatial Algebra Fundamentals
2. mj_comVel() - Body Velocity Computation
3. mj_rne() - Recursive Newton-Euler Algorithm
4. mj_passive() - Passive Forces
5. Key Differences from Simplified Implementations
6. Complete Mathematical Derivation
7. Code-to-Formula Correspondence
8. Common Pitfalls and How MuJoCo Avoids Them
9. Performance Characteristics
10. Integration with Forward Dynamics
11. Verification and Testing

Plus Appendices:
- A: Data Layout Reference
- B: RNE Algorithm Summary
- C: Key Code Snippets

**Best for**: Deep understanding, implementation from scratch, teaching/learning

---

### 4. MUJOCO_DYNAMICS_ANALYSIS.md (12 KB)
Original (supplementary) analysis document.

Contains selected sections for reference.

---

## Quick Navigation

### Looking for...

**A specific function?**
- Check QUICK_REFERENCE.md for file and line numbers
- Then read DYNAMICS_ANALYSIS_SUMMARY.txt for detailed breakdown
- For full context, see MUJOCO_DYNAMICS_DETAILED.md

**How velocity is computed?**
- QUICK_REFERENCE.md: Section "Key Formulas" + "Algorithm Flowchart"
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "2.1 mj_comVel() DETAILED BREAKDOWN"
- MUJOCO_DYNAMICS_DETAILED.md: SECTION 2 (complete walkthrough)

**How RNE algorithm works?**
- QUICK_REFERENCE.md: "RNE() Flow" in Algorithm Flowchart
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "2.2 mj_rne() DETAILED BREAKDOWN"
- MUJOCO_DYNAMICS_DETAILED.md: SECTION 3 (complete analysis)

**Damping/Spring forces?**
- QUICK_REFERENCE.md: "Damping" and "Spring" under Key Formulas
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "2.3 Spring/Damper DETAILED BREAKDOWN"
- MUJOCO_DYNAMICS_DETAILED.md: SECTION 4

**Spatial algebra operations?**
- QUICK_REFERENCE.md: "Spatial Cross-Products"
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "1.3 CRITICAL SPATIAL ALGEBRA OPERATIONS"
- MUJOCO_DYNAMICS_DETAILED.md: SECTION 1

**Code snippets?**
- QUICK_REFERENCE.md: "Critical Code Sections"
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "4. KEY CODE SNIPPETS"
- MUJOCO_DYNAMICS_DETAILED.md: APPENDIX C

**Data structures?**
- QUICK_REFERENCE.md: "Data Structures Cheat Sheet"
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "5. DATA STRUCTURE REFERENCE"
- MUJOCO_DYNAMICS_DETAILED.md: APPENDIX A

**Performance info?**
- QUICK_REFERENCE.md: "Performance Summary"
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "6. PERFORMANCE ANALYSIS"
- MUJOCO_DYNAMICS_DETAILED.md: SECTION 9

**Debugging tips?**
- QUICK_REFERENCE.md: "Debugging Tips"
- DYNAMICS_ANALYSIS_SUMMARY.txt: Section "9. VERIFICATION AND DEBUGGING"
- MUJOCO_DYNAMICS_DETAILED.md: SECTION 11

---

## Source Code Files Analyzed

| File | Lines | Functions |
|------|-------|-----------|
| engine_core_smooth.c | 2355 | mj_comVel, mj_rne, mj_kinematics, mj_comPos |
| engine_passive.c | 1012 | mj_passive, mj_springdamper |
| engine_util_spatial.c | 571 | mju_crossMotion, mju_crossForce, mju_mulInertVec, etc. |
| engine_util_spatial.h | 123 | Function declarations |

**Total analyzed**: 4,061 lines of code

---

## Key Functions Summary

### mj_comVel() - Spatial Velocity Computation
- **File**: engine_core_smooth.c
- **Lines**: 1932-1997 (66 lines)
- **Complexity**: O(nv)
- **Purpose**: Compute spatial velocities and Coriolis terms
- **Key operations**: Forward pass through kinematic tree

### mj_rne() - Recursive Newton-Euler
- **File**: engine_core_smooth.c
- **Lines**: 2068-2119 (52 lines)
- **Complexity**: O(nv)
- **Purpose**: Compute joint torques/forces (inverse dynamics)
- **Key phases**: Forward pass, force computation, backward pass, projection

### mj_passive() - Passive Forces
- **File**: engine_passive.c
- **Lines**: 599-688 (90 lines)
- **Complexity**: O(njnt + ntendon)
- **Purpose**: Compute springs, dampers, gravity compensation
- **Key operations**: DOF damping, joint springs, tendon springs

### Core Spatial Algebra Operations

| Function | File | Lines | Purpose |
|----------|------|-------|---------|
| mju_crossMotion | engine_util_spatial.c | 370-381 | Lie bracket for motion |
| mju_crossForce | engine_util_spatial.c | 385-396 | Lie bracket for force |
| mju_mulInertVec | engine_util_spatial.c | 434-441 | Spatial inertia multiplication |
| mju_mulDofVec | engine_util_spatial.c | 461-469 | Motion subspace multiplication |

---

## Critical Concepts

### 1. Spatial Vectors
```
v[6] = [ω_x, ω_y, ω_z, v_x, v_y, v_z]
       [angular | linear]
```

### 2. Velocity Propagation
```
cvel[i] = cvel[parent] + Σ(cdof[j] * qvel[j])
```

### 3. Coriolis Term
```
cdof_dot[j] = cvel[parent] × cdof[j]  (Lie bracket)
```

### 4. RNE Algorithm
```
Forward: cacc[i] = cacc[parent] + cdof_dot*qvel + cdof*qacc
Forces: cfrc[i] = I[i]*cacc[i] + cvel[i]×(I[i]*cvel[i])
Backward: Accumulate forces up tree
Projection: τ[j] = cdof[j]^T · cfrc_body[body[j]]
```

---

## Implementation Highlights

### Numerical Optimizations
1. **Quaternion normalization** before spring computation
2. **Parent velocity** for cdof_dot (numerically more stable)
3. **Stack allocation** for temporary arrays
4. **Double precision** floating point

### Algorithmic Efficiency
1. **Tree structure** enables O(nv) complexity
2. **Spatial algebra** avoids large matrix operations
3. **Composite frames** use 6 DOF per body (not 12)
4. **Motion subspace** encodes joint structure efficiently

### Numerical Stability
1. Forward-backward recursion respects dependencies
2. Coriolis computed via Lie brackets (consistent with theory)
3. Centrifugal force computed as velocity × momentum
4. All operations in well-defined coordinate frames

---

## How to Use This Documentation

### For Implementation
1. Read QUICK_REFERENCE.md for overview
2. Study MUJOCO_DYNAMICS_DETAILED.md SECTION 2-4
3. Implement mj_comVel() first (simpler)
4. Then implement mj_rne() (more complex)
5. Finally implement mj_passive() (independent)

### For Debugging
1. Check DYNAMICS_ANALYSIS_SUMMARY.txt Section 9
2. Use code snippets from QUICK_REFERENCE.md
3. Verify spatial vector layout
4. Check kinematic tree propagation order
5. Verify Coriolis term computation

### For Understanding
1. Start with QUICK_REFERENCE.md formulas
2. Read MUJOCO_DYNAMICS_DETAILED.md SECTION 5-6 (pitfalls & math)
3. Study SECTION 7 (code correspondence)
4. Check APPENDIX B (RNE summary)

---

## Verification Checklist

Before implementing, verify understanding of:

- [ ] Spatial vector format ([ω | v], not [v | ω])
- [ ] Forward pass through kinematic tree
- [ ] Coriolis computation (Lie bracket, not simple cross-product)
- [ ] Centrifugal force (cvel × (I*cvel))
- [ ] Backward force accumulation
- [ ] Projection to joint space
- [ ] Different handling for FREE/BALL/HINGE/SLIDE joints
- [ ] Quaternion handling in ball joint springs
- [ ] Damping as -b*qvel
- [ ] Spring as -k*(q-q_rest)

---

## Quick Reference: File Paths

### In Repository
```
/mujoco/src/engine/engine_core_smooth.c   (mj_comVel, mj_rne)
/mujoco/src/engine/engine_passive.c       (mj_passive, mj_springdamper)
/mujoco/src/engine/engine_util_spatial.c  (Cross products, operations)
/mujoco/src/engine/engine_util_spatial.h  (Declarations)
```

### Documentation (This Directory)
```
QUICK_REFERENCE.md                        (Quick lookup)
DYNAMICS_ANALYSIS_SUMMARY.txt             (Detailed summary)
MUJOCO_DYNAMICS_DETAILED.md               (Complete analysis)
MUJOCO_ANALYSIS_INDEX.md                  (This file)
```

---

## Key Insights Summary

### Why Spatial Algebra?
- Unified rotation + translation treatment
- Natural Jacobian representation
- Numerically stable Lie bracket operations

### Why Composite Frames?
- 6 DOF per body (not 12)
- Smaller matrices = faster computation
- Automatic COM-based dynamics

### Why RNE?
- O(nv) complexity (not O(n³) matrix inversion)
- Tree structure enables linear scaling
- Natural for forward and inverse dynamics

### Why Motion Subspace?
- Encodes joint structure efficiently
- Coriolis terms via Lie brackets
- Works uniformly for all joint types

---

## Document Statistics

- **Total size**: ~67 KB
- **Total sections**: 40+
- **Code lines analyzed**: 4,061
- **Functions documented**: 20+
- **Mathematical formulas**: 30+
- **Code snippets**: 50+
- **Tables/diagrams**: 15+

---

## Notes

- All line numbers refer to the version in this repository
- Line numbers may shift in newer/older versions of MuJoCo
- All code snippets are directly from source
- Mathematical formulas use standard robotics notation
- Complexity analysis assumes tree-structured kinematic chain

---

## Further Reading

For additional context, see:
- /mujoco/README.md (General MuJoCo documentation)
- /mujoco/include/mujoco/mjmodel.h (Data structure definitions)
- /mujoco/include/mujoco/mjdata.h (Runtime data definitions)

---

Generated: November 2, 2025
Analysis Tool: Claude Code (Haiku 4.5)
Thoroughness: VERY THOROUGH (complete code walkthrough)
Version: 1.0

