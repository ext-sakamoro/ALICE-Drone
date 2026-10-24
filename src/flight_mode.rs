//! Flight mode state machine (`FlightMode` / `TransitionError` / `FlightStateMachine`).

use crate::battery::BatteryStatus;
use crate::geofence::GeofenceStatus;
use crate::vec3::Vec3;

// フライトモードステートマシン
// ============================================================

/// フライトモード
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    /// 待機（地上）
    Idle,
    /// 手動操縦
    Manual,
    /// 自動航行（ウェイポイント追従）
    Auto,
    /// ホバリング（位置保持）
    Hover,
    /// 帰還 (Return-to-Home)
    ReturnToHome,
    /// 着陸
    Landing,
    /// 緊急着陸
    Emergency,
}

/// フライトモード遷移のエラー
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionError {
    /// 許可されない遷移
    InvalidTransition,
    /// バッテリー不足
    LowBattery,
    /// ジオフェンス違反
    GeofenceViolation,
}

/// フライトモードステートマシン
#[derive(Debug, Clone)]
pub struct FlightStateMachine {
    /// 現在のモード
    mode: FlightMode,
    /// ホーム位置
    home_position: Vec3,
    /// 着陸時の目標高度
    landing_altitude: f64,
}

impl FlightStateMachine {
    /// 新しいステートマシンを生成
    #[must_use]
    pub fn new(home_position: Vec3) -> Self {
        Self {
            mode: FlightMode::Idle,
            home_position,
            landing_altitude: 0.0,
        }
    }

    /// 現在のモードを取得
    #[must_use]
    pub const fn mode(&self) -> FlightMode {
        self.mode
    }

    /// ホーム位置を取得
    #[must_use]
    pub const fn home_position(&self) -> Vec3 {
        self.home_position
    }

    /// ホーム位置を更新
    pub fn set_home_position(&mut self, pos: Vec3) {
        self.home_position = pos;
    }

    /// モード遷移を要求
    ///
    /// # Errors
    /// 許可されない遷移の場合は `TransitionError` を返す
    pub fn request_transition(
        &mut self,
        target: FlightMode,
    ) -> Result<FlightMode, TransitionError> {
        if self.is_valid_transition(target) {
            self.mode = target;
            Ok(self.mode)
        } else {
            Err(TransitionError::InvalidTransition)
        }
    }

    /// 遷移が有効かチェック
    #[must_use]
    pub fn is_valid_transition(&self, target: FlightMode) -> bool {
        use FlightMode::{Auto, Emergency, Hover, Idle, Landing, Manual, ReturnToHome};
        matches!(
            (self.mode, target),
            (Idle | Manual | Hover, Auto)
                | (Idle | Auto | Hover, Manual)
                | (Manual | Auto | ReturnToHome | Landing, Hover)
                | (Manual | Auto | Hover, ReturnToHome)
                | (Manual | Auto | Hover | ReturnToHome, Landing)
                | (Landing | Emergency, Idle)
                | (_, Emergency)
        )
    }

    /// バッテリー状態に基づく自動モード遷移
    pub fn auto_transition_for_battery(
        &mut self,
        battery_status: BatteryStatus,
    ) -> Option<FlightMode> {
        match battery_status {
            BatteryStatus::Depleted => {
                self.mode = FlightMode::Emergency;
                Some(FlightMode::Emergency)
            }
            BatteryStatus::Critical => {
                if self.mode != FlightMode::Emergency && self.mode != FlightMode::Landing {
                    self.mode = FlightMode::Landing;
                    Some(FlightMode::Landing)
                } else {
                    None
                }
            }
            BatteryStatus::Low => {
                if self.mode == FlightMode::Auto || self.mode == FlightMode::Manual {
                    self.mode = FlightMode::ReturnToHome;
                    Some(FlightMode::ReturnToHome)
                } else {
                    None
                }
            }
            BatteryStatus::Normal => None,
        }
    }

    /// ジオフェンス状態に基づく自動モード遷移
    pub fn auto_transition_for_geofence(
        &mut self,
        geofence_status: GeofenceStatus,
    ) -> Option<FlightMode> {
        match geofence_status {
            GeofenceStatus::Outside => {
                if self.mode != FlightMode::Emergency
                    && self.mode != FlightMode::ReturnToHome
                    && self.mode != FlightMode::Landing
                {
                    self.mode = FlightMode::ReturnToHome;
                    Some(FlightMode::ReturnToHome)
                } else {
                    None
                }
            }
            GeofenceStatus::Warning | GeofenceStatus::Inside => None,
        }
    }

    /// 着陸中の高度チェック（着陸完了判定）
    pub fn check_landing(&mut self, altitude: f64) -> bool {
        if self.mode == FlightMode::Landing && altitude <= self.landing_altitude + 0.1 {
            self.mode = FlightMode::Idle;
            true
        } else {
            false
        }
    }
}

impl Default for FlightStateMachine {
    fn default() -> Self {
        Self::new(Vec3::zero())
    }
}
