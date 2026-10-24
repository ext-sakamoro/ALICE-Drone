//! PID control (`PidGains` / `PidController` / `QuadPidController` / `QuadOutput`).

// PID制御
// ============================================================

/// PIDコントローラのゲイン設定
#[derive(Debug, Clone, Copy)]
pub struct PidGains {
    /// 比例ゲイン
    pub kp: f64,
    /// 積分ゲイン
    pub ki: f64,
    /// 微分ゲイン
    pub kd: f64,
}

impl PidGains {
    /// 新しいゲインを生成
    #[must_use]
    pub const fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self { kp, ki, kd }
    }
}

/// PIDコントローラ
#[derive(Debug, Clone)]
pub struct PidController {
    /// ゲイン
    gains: PidGains,
    /// 積分値
    integral: f64,
    /// 前回の誤差
    prev_error: f64,
    /// 出力の上下限
    output_limit: f64,
    /// 積分のアンチワインドアップ上限
    integral_limit: f64,
}

impl PidController {
    /// 新しいPIDコントローラを生成
    #[must_use]
    pub fn new(gains: PidGains, output_limit: f64, integral_limit: f64) -> Self {
        Self {
            gains,
            integral: 0.0,
            prev_error: 0.0,
            output_limit: output_limit.abs(),
            integral_limit: integral_limit.abs(),
        }
    }

    /// PID出力を計算
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }
        // 積分項の蓄積
        self.integral += error * dt;
        // アンチワインドアップ
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        // 微分項
        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;
        // PID出力
        let output =
            self.gains.kp * error + self.gains.ki * self.integral + self.gains.kd * derivative;
        output.clamp(-self.output_limit, self.output_limit)
    }

    /// コントローラをリセット
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    /// 現在の積分値を取得
    #[must_use]
    pub fn integral_value(&self) -> f64 {
        self.integral
    }

    /// 現在のゲインを取得
    #[must_use]
    pub const fn gains(&self) -> &PidGains {
        &self.gains
    }

    /// ゲインを更新
    pub fn set_gains(&mut self, gains: PidGains) {
        self.gains = gains;
    }
}

/// 4軸PIDコントローラ（ピッチ、ロール、ヨー、高度）
#[derive(Debug, Clone)]
pub struct QuadPidController {
    /// ピッチ制御
    pub pitch: PidController,
    /// ロール制御
    pub roll: PidController,
    /// ヨー制御
    pub yaw: PidController,
    /// 高度制御
    pub altitude: PidController,
}

/// 4軸の制御出力
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadOutput {
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
    pub altitude: f64,
}

impl QuadPidController {
    /// デフォルトゲインで4軸コントローラを生成
    #[must_use]
    pub fn new() -> Self {
        let angle_gains = PidGains::new(4.0, 0.5, 1.0);
        let yaw_gains = PidGains::new(2.0, 0.2, 0.5);
        let alt_gains = PidGains::new(3.0, 0.3, 1.5);
        Self {
            pitch: PidController::new(angle_gains, 1.0, 5.0),
            roll: PidController::new(angle_gains, 1.0, 5.0),
            yaw: PidController::new(yaw_gains, 1.0, 5.0),
            altitude: PidController::new(alt_gains, 1.0, 10.0),
        }
    }

    /// カスタムゲインで4軸コントローラを生成
    #[must_use]
    pub fn with_gains(
        pitch_gains: PidGains,
        roll_gains: PidGains,
        yaw_gains: PidGains,
        alt_gains: PidGains,
    ) -> Self {
        Self {
            pitch: PidController::new(pitch_gains, 1.0, 5.0),
            roll: PidController::new(roll_gains, 1.0, 5.0),
            yaw: PidController::new(yaw_gains, 1.0, 5.0),
            altitude: PidController::new(alt_gains, 1.0, 10.0),
        }
    }

    /// 4軸同時更新
    pub fn update(&mut self, errors: QuadOutput, dt: f64) -> QuadOutput {
        QuadOutput {
            pitch: self.pitch.update(errors.pitch, dt),
            roll: self.roll.update(errors.roll, dt),
            yaw: self.yaw.update(errors.yaw, dt),
            altitude: self.altitude.update(errors.altitude, dt),
        }
    }

    /// 全軸リセット
    pub fn reset(&mut self) {
        self.pitch.reset();
        self.roll.reset();
        self.yaw.reset();
        self.altitude.reset();
    }
}

impl Default for QuadPidController {
    fn default() -> Self {
        Self::new()
    }
}
