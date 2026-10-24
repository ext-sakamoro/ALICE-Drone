//! Waypoint navigation (`Waypoint` / `NavStatus` / `WaypointNavigator`).

use crate::vec3::Vec3;

// ウェイポイントナビゲーション
// ============================================================

/// ウェイポイント
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// 位置
    pub position: Vec3,
    /// 到達判定半径
    pub radius: f64,
    /// 通過速度（m/s）
    pub speed: f64,
}

impl Waypoint {
    /// 新しいウェイポイントを生成
    #[must_use]
    pub fn new(position: Vec3, radius: f64, speed: f64) -> Self {
        Self {
            position,
            radius: radius.max(0.1),
            speed: speed.max(0.1),
        }
    }
}

/// ナビゲーションの状態
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavStatus {
    /// ナビゲーション中
    Navigating,
    /// 全ウェイポイント到達
    Completed,
    /// ウェイポイント未設定
    NoWaypoints,
}

/// ウェイポイントナビゲータ
#[derive(Debug, Clone)]
pub struct WaypointNavigator {
    /// ウェイポイントリスト
    waypoints: Vec<Waypoint>,
    /// 現在のウェイポイントインデックス
    current_index: usize,
    /// ループモード
    loop_mode: bool,
}

impl WaypointNavigator {
    /// 新しいナビゲータを生成
    #[must_use]
    pub fn new() -> Self {
        Self {
            waypoints: Vec::new(),
            current_index: 0,
            loop_mode: false,
        }
    }

    /// ウェイポイントを追加
    pub fn add_waypoint(&mut self, wp: Waypoint) {
        self.waypoints.push(wp);
    }

    /// ウェイポイントリストをクリア
    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.current_index = 0;
    }

    /// ループモードを設定
    pub fn set_loop_mode(&mut self, enabled: bool) {
        self.loop_mode = enabled;
    }

    /// 現在のターゲットウェイポイントを取得
    #[must_use]
    pub fn current_waypoint(&self) -> Option<&Waypoint> {
        self.waypoints.get(self.current_index)
    }

    /// 現在のインデックスを取得
    #[must_use]
    pub const fn current_index(&self) -> usize {
        self.current_index
    }

    /// ウェイポイント数を取得
    #[must_use]
    pub fn waypoint_count(&self) -> usize {
        self.waypoints.len()
    }

    /// 現在位置に基づいてナビゲーションを更新
    ///
    /// 戻り値：(目標への方向ベクトル（正規化）, 目標速度, ステータス)
    pub fn update(&mut self, current_pos: Vec3) -> (Vec3, f64, NavStatus) {
        if self.waypoints.is_empty() {
            return (Vec3::zero(), 0.0, NavStatus::NoWaypoints);
        }
        if self.current_index >= self.waypoints.len() {
            if self.loop_mode {
                self.current_index = 0;
            } else {
                return (Vec3::zero(), 0.0, NavStatus::Completed);
            }
        }

        let wp = self.waypoints[self.current_index];
        let diff = wp.position.sub(current_pos);
        let dist = diff.length();

        // 到達判定
        if dist <= wp.radius {
            self.current_index += 1;
            if self.current_index >= self.waypoints.len() {
                if self.loop_mode {
                    self.current_index = 0;
                } else {
                    return (Vec3::zero(), 0.0, NavStatus::Completed);
                }
            }
            // 次のウェイポイントへ
            let next_wp = self.waypoints[self.current_index];
            let next_diff = next_wp.position.sub(current_pos);
            return (next_diff.normalized(), next_wp.speed, NavStatus::Navigating);
        }

        (diff.normalized(), wp.speed, NavStatus::Navigating)
    }

    /// インデックスをリセット
    pub fn reset(&mut self) {
        self.current_index = 0;
    }

    /// 全ウェイポイントの総距離を計算
    #[must_use]
    pub fn total_distance(&self) -> f64 {
        if self.waypoints.len() < 2 {
            return 0.0;
        }
        let mut dist = 0.0;
        for i in 1..self.waypoints.len() {
            dist += self.waypoints[i - 1]
                .position
                .distance_to(self.waypoints[i].position);
        }
        dist
    }
}

impl Default for WaypointNavigator {
    fn default() -> Self {
        Self::new()
    }
}
