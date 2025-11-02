//! # 演示程序模块
//!
//! 包含使用多体动力学框架的示例程序

pub mod hinge_chain;

pub use hinge_chain::{create_hinge_chain, CAPSULE_HALF_LENGTH, NUM_CAPSULES};
