//! ALICE-Drone: ドローン/UAV飛行制御ライブラリ
//!
//! PID制御、ウェイポイントナビゲーション、姿勢推定、ジオフェンス、
//! バッテリー管理、フライトモードステートマシンを提供する。

#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::module_name_repetitions,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::unreadable_literal,
    clippy::wildcard_imports,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::should_implement_trait,
    clippy::doc_markdown,
    clippy::imprecise_flops,
    clippy::missing_const_for_fn,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::cast_lossless,
    clippy::many_single_char_names
)]

pub mod attitude;
pub mod battery;
pub mod flight_mode;
pub mod geofence;
pub mod pid;
pub mod prelude;
pub mod vec3;
pub mod waypoint;

#[cfg(test)]
mod integration_tests;

// Backward-compat re-exports.
pub use crate::attitude::*;
pub use crate::battery::*;
pub use crate::flight_mode::*;
pub use crate::geofence::*;
pub use crate::pid::*;
pub use crate::vec3::*;
pub use crate::waypoint::*;
