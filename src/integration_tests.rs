//! Integration tests spanning multiple modules.

#![allow(
    clippy::float_cmp,
    clippy::unreadable_literal,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::explicit_iter_loop,
    clippy::bool_to_int_with_if,
    clippy::approx_constant,
    clippy::cast_lossless,
    clippy::redundant_clone,
    clippy::format_collect,
    clippy::similar_names,
    clippy::needless_collect,
    clippy::iter_cloned_collect,
    clippy::suboptimal_flops,
    clippy::should_panic_without_expect,
    clippy::manual_range_contains
)]

use crate::attitude::*;
use crate::battery::*;
use crate::flight_mode::*;
use crate::geofence::*;
use crate::pid::*;
use crate::vec3::*;
use crate::waypoint::*;

use std::f64::consts::PI;

// --- Vec3テスト ---

#[test]
fn test_vec3_new() {
    let v = Vec3::new(1.0, 2.0, 3.0);
    assert_eq!(v.x, 1.0);
    assert_eq!(v.y, 2.0);
    assert_eq!(v.z, 3.0);
}

#[test]
fn test_vec3_zero() {
    let v = Vec3::zero();
    assert_eq!(v.x, 0.0);
    assert_eq!(v.y, 0.0);
    assert_eq!(v.z, 0.0);
}

#[test]
fn test_vec3_length() {
    let v = Vec3::new(3.0, 4.0, 0.0);
    assert!((v.length() - 5.0).abs() < 1e-10);
}

#[test]
fn test_vec3_length_3d() {
    let v = Vec3::new(1.0, 2.0, 2.0);
    assert!((v.length() - 3.0).abs() < 1e-10);
}

#[test]
fn test_vec3_distance() {
    let a = Vec3::new(0.0, 0.0, 0.0);
    let b = Vec3::new(3.0, 4.0, 0.0);
    assert!((a.distance_to(b) - 5.0).abs() < 1e-10);
}

#[test]
fn test_vec3_normalized() {
    let v = Vec3::new(3.0, 0.0, 0.0);
    let n = v.normalized();
    assert!((n.x - 1.0).abs() < 1e-10);
    assert!(n.y.abs() < 1e-10);
}

#[test]
fn test_vec3_normalized_zero() {
    let v = Vec3::zero();
    let n = v.normalized();
    assert_eq!(n.x, 0.0);
}

#[test]
fn test_vec3_dot() {
    let a = Vec3::new(1.0, 0.0, 0.0);
    let b = Vec3::new(0.0, 1.0, 0.0);
    assert!(a.dot(b).abs() < 1e-10);
}

#[test]
fn test_vec3_dot_parallel() {
    let a = Vec3::new(2.0, 3.0, 4.0);
    assert!((a.dot(a) - a.length() * a.length()).abs() < 1e-10);
}

#[test]
fn test_vec3_cross() {
    let a = Vec3::new(1.0, 0.0, 0.0);
    let b = Vec3::new(0.0, 1.0, 0.0);
    let c = a.cross(b);
    assert!((c.z - 1.0).abs() < 1e-10);
}

#[test]
fn test_vec3_scale() {
    let v = Vec3::new(1.0, 2.0, 3.0).scale(2.0);
    assert_eq!(v.x, 2.0);
    assert_eq!(v.y, 4.0);
    assert_eq!(v.z, 6.0);
}

#[test]
fn test_vec3_add() {
    let r = Vec3::new(1.0, 2.0, 3.0).add(Vec3::new(4.0, 5.0, 6.0));
    assert_eq!(r.x, 5.0);
    assert_eq!(r.y, 7.0);
    assert_eq!(r.z, 9.0);
}

#[test]
fn test_vec3_sub() {
    let r = Vec3::new(4.0, 5.0, 6.0).sub(Vec3::new(1.0, 2.0, 3.0));
    assert_eq!(r.x, 3.0);
    assert_eq!(r.y, 3.0);
    assert_eq!(r.z, 3.0);
}

#[test]
fn test_vec3_horizontal_distance() {
    let a = Vec3::new(0.0, 100.0, 0.0);
    let b = Vec3::new(3.0, 200.0, 4.0);
    assert!((a.horizontal_distance_to(b) - 5.0).abs() < 1e-10);
}

// --- PIDテスト ---

#[test]
fn test_pid_zero_error() {
    let mut pid = PidController::new(PidGains::new(1.0, 0.0, 0.0), 10.0, 10.0);
    let out = pid.update(0.0, 0.01);
    assert_eq!(out, 0.0);
}

#[test]
fn test_pid_proportional() {
    let mut pid = PidController::new(PidGains::new(2.0, 0.0, 0.0), 100.0, 100.0);
    let out = pid.update(5.0, 0.01);
    assert!((out - 10.0).abs() < 1e-10);
}

#[test]
fn test_pid_integral() {
    let mut pid = PidController::new(PidGains::new(0.0, 1.0, 0.0), 100.0, 100.0);
    pid.update(10.0, 1.0);
    let out = pid.update(10.0, 1.0);
    // 積分 = 10*1 + 10*1 = 20
    assert!((out - 20.0).abs() < 1e-10);
}

#[test]
fn test_pid_derivative() {
    let mut pid = PidController::new(PidGains::new(0.0, 0.0, 1.0), 100.0, 100.0);
    pid.update(0.0, 0.01);
    let out = pid.update(10.0, 0.01);
    // 微分 = (10-0)/0.01 = 1000
    assert!((out - 100.0).abs() < 1e-10); // 出力制限100
}

#[test]
fn test_pid_output_clamping() {
    let mut pid = PidController::new(PidGains::new(10.0, 0.0, 0.0), 5.0, 10.0);
    let out = pid.update(100.0, 0.01);
    assert_eq!(out, 5.0);
}

#[test]
fn test_pid_negative_clamping() {
    let mut pid = PidController::new(PidGains::new(10.0, 0.0, 0.0), 5.0, 10.0);
    let out = pid.update(-100.0, 0.01);
    assert_eq!(out, -5.0);
}

#[test]
fn test_pid_integral_windup() {
    let mut pid = PidController::new(PidGains::new(0.0, 1.0, 0.0), 100.0, 2.0);
    for _ in 0..100 {
        pid.update(10.0, 1.0);
    }
    // 積分は2.0に制限される
    assert!((pid.integral_value() - 2.0).abs() < 1e-10);
}

#[test]
fn test_pid_zero_dt() {
    let mut pid = PidController::new(PidGains::new(1.0, 1.0, 1.0), 10.0, 10.0);
    let out = pid.update(5.0, 0.0);
    assert_eq!(out, 0.0);
}

#[test]
fn test_pid_reset() {
    let mut pid = PidController::new(PidGains::new(1.0, 1.0, 1.0), 100.0, 100.0);
    pid.update(10.0, 1.0);
    pid.reset();
    assert_eq!(pid.integral_value(), 0.0);
}

#[test]
fn test_pid_set_gains() {
    let mut pid = PidController::new(PidGains::new(1.0, 0.0, 0.0), 10.0, 10.0);
    pid.set_gains(PidGains::new(5.0, 0.0, 0.0));
    assert_eq!(pid.gains().kp, 5.0);
}

#[test]
fn test_pid_convergence() {
    // PIDで値が目標に収束するか
    let mut pid = PidController::new(PidGains::new(1.0, 0.1, 0.5), 100.0, 100.0);
    let target = 10.0;
    let mut value = 0.0;
    let dt = 0.01;
    for _ in 0..1000 {
        let error = target - value;
        let correction = pid.update(error, dt);
        value += correction * dt;
    }
    assert!((value - target).abs() < 1.0);
}

// --- QuadPIDテスト ---

#[test]
fn test_quad_pid_default() {
    let qp = QuadPidController::new();
    assert_eq!(qp.pitch.gains().kp, 4.0);
}

#[test]
fn test_quad_pid_update() {
    let mut qp = QuadPidController::new();
    let errors = QuadOutput {
        pitch: 0.1,
        roll: -0.1,
        yaw: 0.05,
        altitude: 0.5,
    };
    let out = qp.update(errors, 0.01);
    assert!(out.pitch > 0.0);
    assert!(out.roll < 0.0);
}

#[test]
fn test_quad_pid_reset() {
    let mut qp = QuadPidController::new();
    let errors = QuadOutput {
        pitch: 1.0,
        roll: 1.0,
        yaw: 1.0,
        altitude: 1.0,
    };
    qp.update(errors, 0.01);
    qp.reset();
    assert_eq!(qp.pitch.integral_value(), 0.0);
    assert_eq!(qp.altitude.integral_value(), 0.0);
}

#[test]
fn test_quad_pid_with_gains() {
    let g = PidGains::new(1.0, 0.1, 0.5);
    let qp = QuadPidController::with_gains(g, g, g, g);
    assert_eq!(qp.yaw.gains().kp, 1.0);
}

// --- クォータニオンテスト ---

#[test]
fn test_quaternion_identity() {
    let q = Quaternion::identity();
    assert_eq!(q.w, 1.0);
    assert_eq!(q.x, 0.0);
}

#[test]
fn test_quaternion_norm() {
    let q = Quaternion::identity();
    assert!((q.norm() - 1.0).abs() < 1e-10);
}

#[test]
fn test_quaternion_normalize() {
    let q = Quaternion::new(2.0, 0.0, 0.0, 0.0).normalized();
    assert!((q.w - 1.0).abs() < 1e-10);
}

#[test]
fn test_quaternion_conjugate() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let c = q.conjugate();
    assert_eq!(c.w, 1.0);
    assert_eq!(c.x, -2.0);
}

#[test]
fn test_quaternion_multiply_identity() {
    let q = Quaternion::from_euler(0.1, 0.2, 0.3);
    let r = q.multiply(Quaternion::identity());
    assert!((r.w - q.w).abs() < 1e-10);
}

#[test]
fn test_quaternion_from_axis_angle() {
    let q = Quaternion::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), PI / 2.0);
    assert!((q.norm() - 1.0).abs() < 1e-10);
}

#[test]
fn test_quaternion_euler_roundtrip() {
    let roll = 0.3;
    let pitch = 0.2;
    let yaw = 0.1;
    let q = Quaternion::from_euler(roll, pitch, yaw);
    let (r, p, y) = q.to_euler();
    assert!((r - roll).abs() < 1e-10);
    assert!((p - pitch).abs() < 1e-10);
    assert!((y - yaw).abs() < 1e-10);
}

#[test]
fn test_quaternion_rotate_vector() {
    // Y軸周りに90度回転 → X軸ベクトルがZ軸方向に
    let q = Quaternion::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), PI / 2.0);
    let v = q.rotate_vector(Vec3::new(1.0, 0.0, 0.0));
    assert!(v.x.abs() < 1e-10);
    assert!((v.z - (-1.0)).abs() < 1e-10);
}

#[test]
fn test_quaternion_slerp_endpoints() {
    let q1 = Quaternion::from_euler(0.0, 0.0, 0.0);
    let q2 = Quaternion::from_euler(0.5, 0.0, 0.0);
    let r0 = q1.slerp(q2, 0.0);
    let r1 = q1.slerp(q2, 1.0);
    assert!((r0.w - q1.w).abs() < 1e-6);
    assert!((r1.w - q2.w).abs() < 1e-6);
}

#[test]
fn test_quaternion_slerp_midpoint() {
    let q1 = Quaternion::from_euler(0.0, 0.0, 0.0);
    let q2 = Quaternion::from_euler(1.0, 0.0, 0.0);
    let mid = q1.slerp(q2, 0.5);
    let (r, _, _) = mid.to_euler();
    assert!((r - 0.5).abs() < 1e-3);
}

#[test]
fn test_quaternion_double_rotation() {
    // Y軸周りにPI/4を2回回転 → PI/2の回転
    let q1 = Quaternion::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), PI / 4.0);
    let q2 = q1.multiply(q1);
    // 回転角度を検証（ベクトル回転で確認）
    let v = q2.rotate_vector(Vec3::new(1.0, 0.0, 0.0));
    // X軸ベクトルがPI/2回転でZ軸負方向に向く
    assert!(v.x.abs() < 1e-6);
    assert!((v.z - (-1.0)).abs() < 1e-6);
}

// --- 相補フィルタテスト ---

#[test]
fn test_complementary_filter_init() {
    let cf = ComplementaryFilter::new(0.02);
    let q = cf.attitude();
    assert!((q.w - 1.0).abs() < 1e-10);
}

#[test]
fn test_complementary_filter_gyro_only() {
    let mut cf = ComplementaryFilter::new(0.0); // 加速度無視
    let gyro = Vec3::new(0.0, 0.0, 1.0); // ヨー回転
    let accel = Vec3::new(0.0, 0.0, -9.81);
    cf.update(gyro, accel, 0.1);
    let q = cf.attitude();
    assert!(q.w < 1.0); // 回転あり
}

#[test]
fn test_complementary_filter_reset() {
    let mut cf = ComplementaryFilter::new(0.02);
    cf.update(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -9.81), 0.1);
    cf.reset();
    let q = cf.attitude();
    assert!((q.w - 1.0).abs() < 1e-10);
}

#[test]
fn test_complementary_filter_euler() {
    let cf = ComplementaryFilter::new(0.02);
    let (r, p, y) = cf.euler_angles();
    assert!(r.abs() < 1e-10);
    assert!(p.abs() < 1e-10);
    assert!(y.abs() < 1e-10);
}

#[test]
fn test_complementary_filter_alpha_clamp() {
    let cf = ComplementaryFilter::new(2.0);
    assert!(cf.alpha <= 1.0);
}

// --- ウェイポイントテスト ---

#[test]
fn test_waypoint_new() {
    let wp = Waypoint::new(Vec3::new(10.0, 5.0, 20.0), 2.0, 5.0);
    assert_eq!(wp.position.x, 10.0);
    assert_eq!(wp.radius, 2.0);
}

#[test]
fn test_waypoint_min_radius() {
    let wp = Waypoint::new(Vec3::zero(), -1.0, 1.0);
    assert!(wp.radius >= 0.1);
}

#[test]
fn test_navigator_empty() {
    let mut nav = WaypointNavigator::new();
    let (_, _, status) = nav.update(Vec3::zero());
    assert_eq!(status, NavStatus::NoWaypoints);
}

#[test]
fn test_navigator_add() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::new(10.0, 0.0, 0.0), 1.0, 5.0));
    assert_eq!(nav.waypoint_count(), 1);
}

#[test]
fn test_navigator_single_waypoint() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::new(10.0, 0.0, 0.0), 1.0, 5.0));
    let (dir, speed, status) = nav.update(Vec3::zero());
    assert_eq!(status, NavStatus::Navigating);
    assert!(dir.x > 0.0);
    assert_eq!(speed, 5.0);
}

#[test]
fn test_navigator_reach_waypoint() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::new(10.0, 0.0, 0.0), 2.0, 5.0));
    let (_, _, status) = nav.update(Vec3::new(10.0, 0.0, 0.0));
    assert_eq!(status, NavStatus::Completed);
}

#[test]
fn test_navigator_multiple_waypoints() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::new(5.0, 0.0, 0.0), 2.0, 5.0));
    nav.add_waypoint(Waypoint::new(Vec3::new(15.0, 0.0, 0.0), 2.0, 3.0));

    // 最初のウェイポイントに到達
    let (_, _, status) = nav.update(Vec3::new(5.0, 0.0, 0.0));
    assert_eq!(status, NavStatus::Navigating);
    assert_eq!(nav.current_index(), 1);
}

#[test]
fn test_navigator_loop_mode() {
    let mut nav = WaypointNavigator::new();
    nav.set_loop_mode(true);
    nav.add_waypoint(Waypoint::new(Vec3::new(5.0, 0.0, 0.0), 2.0, 5.0));

    // 到達 → ループしてインデックス0に戻る
    let (_, _, status) = nav.update(Vec3::new(5.0, 0.0, 0.0));
    assert_eq!(status, NavStatus::Navigating);
    assert_eq!(nav.current_index(), 0);
}

#[test]
fn test_navigator_clear() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::zero(), 1.0, 1.0));
    nav.clear();
    assert_eq!(nav.waypoint_count(), 0);
}

#[test]
fn test_navigator_reset() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::new(1.0, 0.0, 0.0), 2.0, 1.0));
    nav.add_waypoint(Waypoint::new(Vec3::new(5.0, 0.0, 0.0), 2.0, 1.0));
    nav.update(Vec3::new(1.0, 0.0, 0.0)); // 最初に到達
    nav.reset();
    assert_eq!(nav.current_index(), 0);
}

#[test]
fn test_navigator_total_distance() {
    let mut nav = WaypointNavigator::new();
    nav.add_waypoint(Waypoint::new(Vec3::new(0.0, 0.0, 0.0), 1.0, 1.0));
    nav.add_waypoint(Waypoint::new(Vec3::new(3.0, 4.0, 0.0), 1.0, 1.0));
    assert!((nav.total_distance() - 5.0).abs() < 1e-10);
}

#[test]
fn test_navigator_total_distance_empty() {
    let nav = WaypointNavigator::new();
    assert_eq!(nav.total_distance(), 0.0);
}

#[test]
fn test_navigator_current_waypoint() {
    let mut nav = WaypointNavigator::new();
    assert!(nav.current_waypoint().is_none());
    nav.add_waypoint(Waypoint::new(Vec3::new(10.0, 0.0, 0.0), 1.0, 5.0));
    assert!(nav.current_waypoint().is_some());
}

// --- ジオフェンステスト ---

#[test]
fn test_geofence_cylinder_inside() {
    let gf = Geofence::cylinder(Vec3::zero(), 100.0, 0.0, 50.0, 5.0);
    let status = gf.check(Vec3::new(10.0, 25.0, 10.0));
    assert_eq!(status, GeofenceStatus::Inside);
}

#[test]
fn test_geofence_cylinder_warning() {
    let gf = Geofence::cylinder(Vec3::zero(), 100.0, 0.0, 50.0, 5.0);
    // 境界付近
    let status = gf.check(Vec3::new(97.0, 25.0, 0.0));
    assert_eq!(status, GeofenceStatus::Warning);
}

#[test]
fn test_geofence_cylinder_outside() {
    let gf = Geofence::cylinder(Vec3::zero(), 100.0, 0.0, 50.0, 5.0);
    let status = gf.check(Vec3::new(200.0, 25.0, 0.0));
    assert_eq!(status, GeofenceStatus::Outside);
}

#[test]
fn test_geofence_cylinder_altitude_outside() {
    let gf = Geofence::cylinder(Vec3::zero(), 100.0, 0.0, 50.0, 5.0);
    let status = gf.check(Vec3::new(0.0, 60.0, 0.0));
    assert_eq!(status, GeofenceStatus::Outside);
}

#[test]
fn test_geofence_cylinder_altitude_warning() {
    let gf = Geofence::cylinder(Vec3::zero(), 100.0, 0.0, 50.0, 5.0);
    let status = gf.check(Vec3::new(0.0, 47.0, 0.0));
    assert_eq!(status, GeofenceStatus::Warning);
}

#[test]
fn test_geofence_box_inside() {
    let gf = Geofence::bounding_box(
        Vec3::new(-50.0, 0.0, -50.0),
        Vec3::new(50.0, 100.0, 50.0),
        5.0,
    );
    let status = gf.check(Vec3::new(0.0, 50.0, 0.0));
    assert_eq!(status, GeofenceStatus::Inside);
}

#[test]
fn test_geofence_box_outside() {
    let gf = Geofence::bounding_box(
        Vec3::new(-50.0, 0.0, -50.0),
        Vec3::new(50.0, 100.0, 50.0),
        5.0,
    );
    let status = gf.check(Vec3::new(100.0, 50.0, 0.0));
    assert_eq!(status, GeofenceStatus::Outside);
}

#[test]
fn test_geofence_box_warning() {
    let gf = Geofence::bounding_box(
        Vec3::new(-50.0, 0.0, -50.0),
        Vec3::new(50.0, 100.0, 50.0),
        5.0,
    );
    let status = gf.check(Vec3::new(47.0, 50.0, 0.0));
    assert_eq!(status, GeofenceStatus::Warning);
}

#[test]
fn test_geofence_distance_to_boundary_inside() {
    let gf = Geofence::bounding_box(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(100.0, 100.0, 100.0),
        5.0,
    );
    let d = gf.distance_to_boundary(Vec3::new(10.0, 50.0, 50.0));
    assert!((d - 10.0).abs() < 1e-10);
}

#[test]
fn test_geofence_distance_to_boundary_outside() {
    let gf = Geofence::bounding_box(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(100.0, 100.0, 100.0),
        5.0,
    );
    let d = gf.distance_to_boundary(Vec3::new(-5.0, 50.0, 50.0));
    assert!(d < 0.0);
}

#[test]
fn test_geofence_cylinder_distance() {
    let gf = Geofence::cylinder(Vec3::zero(), 100.0, 0.0, 50.0, 5.0);
    let d = gf.distance_to_boundary(Vec3::new(0.0, 25.0, 0.0));
    // 水平距離=0 → 半径方向100, 高度下限25, 高度上限25 → min=25
    assert!((d - 25.0).abs() < 1e-10);
}

// --- バッテリーテスト ---

#[test]
fn test_battery_new() {
    let bm = BatteryManager::new(5000.0, 16.8);
    assert_eq!(bm.state_of_charge(), 1.0);
    assert_eq!(bm.status(), BatteryStatus::Normal);
}

#[test]
fn test_battery_consume() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    bm.consume(1000.0, 3600.0); // 1A × 1h = 1000mAh
    assert!(bm.state_of_charge() < 0.01);
}

#[test]
fn test_battery_status_low() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    // 消費して残り25%にする（低残量閾値30%以下、緊急閾値15%以上）
    bm.consume(750.0, 3600.0); // 750mAh消費 → 残り250mAh = 25%
    assert_eq!(bm.status(), BatteryStatus::Low);
}

#[test]
fn test_battery_status_critical() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    bm.consume(900.0 * 3600.0 / 3600.0, 3600.0);
    // 残り100mAh = 10%
    assert_eq!(bm.status(), BatteryStatus::Critical);
}

#[test]
fn test_battery_status_depleted() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    bm.consume(2000.0 * 3600.0 / 3600.0, 3600.0);
    assert_eq!(bm.status(), BatteryStatus::Depleted);
}

#[test]
fn test_battery_reset() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    bm.consume(500.0 * 3600.0 / 3600.0, 3600.0);
    bm.reset();
    assert_eq!(bm.state_of_charge(), 1.0);
}

#[test]
fn test_battery_set_thresholds() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    bm.set_thresholds(0.5, 0.2);
    bm.consume(600.0 * 3600.0 / 3600.0, 3600.0);
    // 残り400mAh = 40% → Low (50%閾値)
    assert_eq!(bm.status(), BatteryStatus::Low);
}

#[test]
fn test_battery_should_return_home() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    // 通常状態では帰還不要
    assert!(!bm.should_return_home(100.0, 5.0));
    // バッテリー低下時は帰還必要
    bm.consume(900.0 * 3600.0 / 3600.0, 3600.0);
    assert!(bm.should_return_home(100.0, 5.0));
}

#[test]
fn test_battery_estimated_flight_time() {
    let bm = BatteryManager::new(1000.0, 16.8);
    // 消費レート0の場合は無限
    assert_eq!(bm.estimated_flight_time(), f64::INFINITY);
}

#[test]
fn test_battery_voltage() {
    let bm = BatteryManager::new(1000.0, 16.8);
    assert_eq!(bm.voltage(), 16.8);
}

#[test]
fn test_battery_remaining() {
    let bm = BatteryManager::new(1000.0, 16.8);
    assert_eq!(bm.remaining(), 1000.0);
}

#[test]
fn test_battery_zero_dt() {
    let mut bm = BatteryManager::new(1000.0, 16.8);
    bm.consume(1000.0, 0.0);
    assert_eq!(bm.state_of_charge(), 1.0);
}

#[test]
fn test_battery_zero_capacity() {
    let bm = BatteryManager::new(0.0, 16.8);
    assert_eq!(bm.state_of_charge(), 0.0);
}

// --- フライトモードテスト ---

#[test]
fn test_flight_mode_initial() {
    let fsm = FlightStateMachine::new(Vec3::zero());
    assert_eq!(fsm.mode(), FlightMode::Idle);
}

#[test]
fn test_flight_mode_idle_to_manual() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    let result = fsm.request_transition(FlightMode::Manual);
    assert_eq!(result, Ok(FlightMode::Manual));
}

#[test]
fn test_flight_mode_idle_to_auto() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    let result = fsm.request_transition(FlightMode::Auto);
    assert_eq!(result, Ok(FlightMode::Auto));
}

#[test]
fn test_flight_mode_invalid_transition() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    // IdleからHoverは不可
    let result = fsm.request_transition(FlightMode::Hover);
    assert_eq!(result, Err(TransitionError::InvalidTransition));
}

#[test]
fn test_flight_mode_manual_to_auto() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    let result = fsm.request_transition(FlightMode::Auto);
    assert_eq!(result, Ok(FlightMode::Auto));
}

#[test]
fn test_flight_mode_manual_to_rth() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    let result = fsm.request_transition(FlightMode::ReturnToHome);
    assert_eq!(result, Ok(FlightMode::ReturnToHome));
}

#[test]
fn test_flight_mode_emergency_from_any() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    let result = fsm.request_transition(FlightMode::Emergency);
    assert_eq!(result, Ok(FlightMode::Emergency));
}

#[test]
fn test_flight_mode_emergency_to_idle() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Emergency).unwrap();
    let result = fsm.request_transition(FlightMode::Idle);
    assert_eq!(result, Ok(FlightMode::Idle));
}

#[test]
fn test_flight_mode_emergency_to_manual_invalid() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Emergency).unwrap();
    let result = fsm.request_transition(FlightMode::Manual);
    assert_eq!(result, Err(TransitionError::InvalidTransition));
}

#[test]
fn test_flight_mode_landing_to_idle() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    fsm.request_transition(FlightMode::Landing).unwrap();
    let result = fsm.request_transition(FlightMode::Idle);
    assert_eq!(result, Ok(FlightMode::Idle));
}

#[test]
fn test_flight_mode_auto_battery_depleted() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_battery(BatteryStatus::Depleted);
    assert_eq!(mode, Some(FlightMode::Emergency));
}

#[test]
fn test_flight_mode_auto_battery_critical() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_battery(BatteryStatus::Critical);
    assert_eq!(mode, Some(FlightMode::Landing));
}

#[test]
fn test_flight_mode_auto_battery_low() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_battery(BatteryStatus::Low);
    assert_eq!(mode, Some(FlightMode::ReturnToHome));
}

#[test]
fn test_flight_mode_auto_battery_normal() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_battery(BatteryStatus::Normal);
    assert_eq!(mode, None);
}

#[test]
fn test_flight_mode_geofence_outside() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_geofence(GeofenceStatus::Outside);
    assert_eq!(mode, Some(FlightMode::ReturnToHome));
}

#[test]
fn test_flight_mode_geofence_warning() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_geofence(GeofenceStatus::Warning);
    assert_eq!(mode, None);
}

#[test]
fn test_flight_mode_geofence_inside() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();
    let mode = fsm.auto_transition_for_geofence(GeofenceStatus::Inside);
    assert_eq!(mode, None);
}

#[test]
fn test_flight_mode_check_landing_complete() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    fsm.request_transition(FlightMode::Landing).unwrap();
    assert!(fsm.check_landing(0.05));
    assert_eq!(fsm.mode(), FlightMode::Idle);
}

#[test]
fn test_flight_mode_check_landing_not_done() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    fsm.request_transition(FlightMode::Landing).unwrap();
    assert!(!fsm.check_landing(5.0));
    assert_eq!(fsm.mode(), FlightMode::Landing);
}

#[test]
fn test_flight_mode_home_position() {
    let home = Vec3::new(10.0, 0.0, 20.0);
    let mut fsm = FlightStateMachine::new(home);
    assert_eq!(fsm.home_position().x, 10.0);
    fsm.set_home_position(Vec3::new(5.0, 0.0, 5.0));
    assert_eq!(fsm.home_position().x, 5.0);
}

#[test]
fn test_flight_mode_default() {
    let fsm = FlightStateMachine::default();
    assert_eq!(fsm.mode(), FlightMode::Idle);
}

#[test]
fn test_flight_mode_is_valid_transition() {
    let fsm = FlightStateMachine::new(Vec3::zero());
    assert!(fsm.is_valid_transition(FlightMode::Manual));
    assert!(!fsm.is_valid_transition(FlightMode::Landing));
}

#[test]
fn test_flight_mode_rth_to_hover() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    fsm.request_transition(FlightMode::ReturnToHome).unwrap();
    let result = fsm.request_transition(FlightMode::Hover);
    assert_eq!(result, Ok(FlightMode::Hover));
}

#[test]
fn test_flight_mode_hover_to_landing() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    fsm.request_transition(FlightMode::Hover).unwrap();
    let result = fsm.request_transition(FlightMode::Landing);
    assert_eq!(result, Ok(FlightMode::Landing));
}

#[test]
fn test_flight_mode_landing_to_hover() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Manual).unwrap();
    fsm.request_transition(FlightMode::Landing).unwrap();
    let result = fsm.request_transition(FlightMode::Hover);
    assert_eq!(result, Ok(FlightMode::Hover));
}

// --- 統合テスト ---

#[test]
fn test_integration_full_flight() {
    // フルフライトシミュレーション
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    let mut nav = WaypointNavigator::new();
    let mut battery = BatteryManager::new(5000.0, 16.8);
    let gf = Geofence::cylinder(Vec3::zero(), 500.0, 0.0, 120.0, 20.0);

    // ウェイポイント設定
    nav.add_waypoint(Waypoint::new(Vec3::new(100.0, 50.0, 0.0), 5.0, 10.0));
    nav.add_waypoint(Waypoint::new(Vec3::new(200.0, 50.0, 100.0), 5.0, 10.0));

    // 離陸
    fsm.request_transition(FlightMode::Auto).unwrap();
    assert_eq!(fsm.mode(), FlightMode::Auto);

    // ジオフェンス確認
    let pos = Vec3::new(50.0, 30.0, 0.0);
    assert_eq!(gf.check(pos), GeofenceStatus::Inside);

    // ナビゲーション
    let (dir, speed, status) = nav.update(pos);
    assert_eq!(status, NavStatus::Navigating);
    assert!(speed > 0.0);
    assert!(dir.length() > 0.0);

    // バッテリー消費
    battery.consume(10000.0, 60.0);
    assert_eq!(battery.status(), BatteryStatus::Normal);
}

#[test]
fn test_integration_emergency_sequence() {
    let mut fsm = FlightStateMachine::new(Vec3::zero());
    fsm.request_transition(FlightMode::Auto).unwrap();

    // ジオフェンス違反 → RTH
    fsm.auto_transition_for_geofence(GeofenceStatus::Outside);
    assert_eq!(fsm.mode(), FlightMode::ReturnToHome);

    // バッテリー緊急 → Emergency
    fsm.auto_transition_for_battery(BatteryStatus::Depleted);
    assert_eq!(fsm.mode(), FlightMode::Emergency);
}

#[test]
fn test_integration_pid_with_attitude() {
    let mut qpid = QuadPidController::new();
    let cf = ComplementaryFilter::new(0.02);
    let (roll, pitch, _yaw) = cf.euler_angles();

    let errors = QuadOutput {
        pitch: 0.0 - pitch,
        roll: 0.0 - roll,
        yaw: 0.0,
        altitude: 10.0,
    };
    let output = qpid.update(errors, 0.01);
    // 初期状態ではエラーは高度のみ
    assert!(output.altitude > 0.0);
}
