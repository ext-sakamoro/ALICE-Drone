//! Battery management (`BatteryStatus` / `BatteryManager`).

// バッテリー管理
// ============================================================

/// バッテリーの状態
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryStatus {
    /// 正常
    Normal,
    /// 低残量警告
    Low,
    /// 緊急（即時着陸が必要）
    Critical,
    /// 放電完了
    Depleted,
}

/// バッテリーマネージャ
#[derive(Debug, Clone)]
pub struct BatteryManager {
    /// 最大容量 (mAh)
    capacity: f64,
    /// 現在の残量 (mAh)
    remaining: f64,
    /// 電圧 (V)
    voltage: f64,
    /// 最小安全電圧
    min_voltage: f64,
    /// 低残量閾値（割合）
    low_threshold: f64,
    /// 緊急閾値（割合）
    critical_threshold: f64,
    /// 消費履歴（直近の消費レート mAh/s）
    consumption_rate: f64,
}

impl BatteryManager {
    /// 新しいバッテリーマネージャを生成
    #[must_use]
    pub fn new(capacity: f64, voltage: f64) -> Self {
        Self {
            capacity,
            remaining: capacity,
            voltage,
            min_voltage: 3.3 * 4.0, // 4セルLiPo最小電圧
            low_threshold: 0.30,
            critical_threshold: 0.15,
            consumption_rate: 0.0,
        }
    }

    /// 閾値を設定
    pub fn set_thresholds(&mut self, low: f64, critical: f64) {
        self.low_threshold = low.clamp(0.0, 1.0);
        self.critical_threshold = critical.clamp(0.0, self.low_threshold);
    }

    /// バッテリー消費を更新
    pub fn consume(&mut self, current_ma: f64, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let consumed = current_ma * dt / 3600.0; // mAh
        self.remaining = (self.remaining - consumed).max(0.0);
        self.consumption_rate = current_ma / 3600.0;
        // 電圧降下のシミュレーション（簡易線形モデル）
        let soc = self.state_of_charge();
        self.voltage = self.min_voltage + (self.voltage_full() - self.min_voltage) * soc;
    }

    /// 充電状態（0.0〜1.0）
    #[must_use]
    pub fn state_of_charge(&self) -> f64 {
        if self.capacity <= 0.0 {
            return 0.0;
        }
        (self.remaining / self.capacity).clamp(0.0, 1.0)
    }

    /// バッテリー状態を取得
    #[must_use]
    pub fn status(&self) -> BatteryStatus {
        let soc = self.state_of_charge();
        if soc <= 0.01 {
            BatteryStatus::Depleted
        } else if soc <= self.critical_threshold {
            BatteryStatus::Critical
        } else if soc <= self.low_threshold {
            BatteryStatus::Low
        } else {
            BatteryStatus::Normal
        }
    }

    /// 帰還に必要かチェック（残量が閾値以下でRTH推奨）
    #[must_use]
    pub fn should_return_home(&self, distance_to_home: f64, speed: f64) -> bool {
        if speed <= 0.0 || self.consumption_rate <= 0.0 {
            return self.status() != BatteryStatus::Normal;
        }
        let time_to_home = distance_to_home / speed;
        let needed = self.consumption_rate * time_to_home * 1.3; // 30%マージン
        self.remaining <= needed || self.status() == BatteryStatus::Critical
    }

    /// 推定飛行可能時間（秒）
    #[must_use]
    pub fn estimated_flight_time(&self) -> f64 {
        if self.consumption_rate <= 0.0 {
            return f64::INFINITY;
        }
        self.remaining / self.consumption_rate
    }

    /// 現在の電圧を取得
    #[must_use]
    pub const fn voltage(&self) -> f64 {
        self.voltage
    }

    /// 満充電電圧（4セルLiPoの場合）
    #[must_use]
    pub fn voltage_full(&self) -> f64 {
        4.2 * 4.0 // 16.8V
    }

    /// 残量を取得 (mAh)
    #[must_use]
    pub const fn remaining(&self) -> f64 {
        self.remaining
    }

    /// バッテリーをリセット（満充電に戻す）
    pub fn reset(&mut self) {
        self.remaining = self.capacity;
        self.voltage = self.voltage_full();
        self.consumption_rate = 0.0;
    }
}
