//! Quaternion + attitude estimation (`Quaternion` / `ComplementaryFilter`).

use crate::vec3::Vec3;

// クォータニオン・姿勢推定
// ============================================================

/// クォータニオン（姿勢表現用）
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    /// 単位クォータニオンを生成
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// 成分を指定して生成
    #[must_use]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// 軸角度からクォータニオンを生成
    #[must_use]
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let half = angle / 2.0;
        let s = half.sin();
        let a = axis.normalized();
        Self {
            w: half.cos(),
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
        }
    }

    /// オイラー角（ロール、ピッチ、ヨー）からクォータニオンを生成
    #[must_use]
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cr = (roll / 2.0).cos();
        let sr = (roll / 2.0).sin();
        let cp = (pitch / 2.0).cos();
        let sp = (pitch / 2.0).sin();
        let cy = (yaw / 2.0).cos();
        let sy = (yaw / 2.0).sin();
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// ノルム
    #[must_use]
    pub fn norm(self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// 正規化
    #[must_use]
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n < 1e-12 {
            return Self::identity();
        }
        Self {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    /// 共役
    #[must_use]
    pub const fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// クォータニオン乗算
    #[must_use]
    pub fn multiply(self, other: Self) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    /// オイラー角に変換（ロール、ピッチ、ヨー）
    #[must_use]
    pub fn to_euler(self) -> (f64, f64, f64) {
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f64::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    /// ベクトルを回転
    #[must_use]
    pub fn rotate_vector(self, v: Vec3) -> Vec3 {
        let qv = Self::new(0.0, v.x, v.y, v.z);
        let result = self.multiply(qv).multiply(self.conjugate());
        Vec3::new(result.x, result.y, result.z)
    }

    /// 球面線形補間
    #[must_use]
    pub fn slerp(self, other: Self, t: f64) -> Self {
        let mut dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z;
        let other = if dot < 0.0 {
            dot = -dot;
            Self::new(-other.w, -other.x, -other.y, -other.z)
        } else {
            other
        };

        if dot > 0.9995 {
            // ほぼ同じ向き：線形補間
            return Self::new(
                self.w + t * (other.w - self.w),
                self.x + t * (other.x - self.x),
                self.y + t * (other.y - self.y),
                self.z + t * (other.z - self.z),
            )
            .normalized();
        }

        let theta = dot.clamp(-1.0, 1.0).acos();
        let sin_theta = theta.sin();
        if sin_theta.abs() < 1e-12 {
            return self;
        }
        let a = ((1.0 - t) * theta).sin() / sin_theta;
        let b = (t * theta).sin() / sin_theta;
        Self::new(
            a * self.w + b * other.w,
            a * self.x + b * other.x,
            a * self.y + b * other.y,
            a * self.z + b * other.z,
        )
        .normalized()
    }
}

/// 相補フィルタによる姿勢推定器
#[derive(Debug, Clone)]
pub struct ComplementaryFilter {
    /// 加速度計の重み（0.0〜1.0）
    pub(crate) alpha: f64,
    /// 現在のクォータニオン姿勢
    attitude: Quaternion,
}

impl ComplementaryFilter {
    /// 新しい相補フィルタを生成（alpha: 加速度計の重み）
    #[must_use]
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            attitude: Quaternion::identity(),
        }
    }

    /// ジャイロと加速度計のデータで姿勢を更新
    ///
    /// - `gyro`: 角速度 (rad/s) [x=roll_rate, y=pitch_rate, z=yaw_rate]
    /// - `accel`: 加速度計測値（正規化不要、内部で正規化）
    /// - `dt`: 時間ステップ (s)
    pub fn update(&mut self, gyro: Vec3, accel: Vec3, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        // ジャイロによる姿勢積分
        let angle = gyro.length() * dt;
        if angle > 1e-12 {
            let axis = gyro.normalized();
            let dq = Quaternion::from_axis_angle(axis, angle);
            self.attitude = self.attitude.multiply(dq).normalized();
        }

        // 加速度計からロール・ピッチを推定
        let an = accel.normalized();
        if an.length() > 0.5 {
            let accel_roll = an.y.atan2(an.z);
            let accel_pitch = (-an.x).atan2((an.y * an.y + an.z * an.z).sqrt());
            let (gyro_roll, gyro_pitch, gyro_yaw) = self.attitude.to_euler();

            // 相補フィルタで統合
            let fused_roll = (1.0 - self.alpha) * gyro_roll + self.alpha * accel_roll;
            let fused_pitch = (1.0 - self.alpha) * gyro_pitch + self.alpha * accel_pitch;
            self.attitude = Quaternion::from_euler(fused_roll, fused_pitch, gyro_yaw);
        }
    }

    /// 現在の姿勢クォータニオンを取得
    #[must_use]
    pub const fn attitude(&self) -> Quaternion {
        self.attitude
    }

    /// 姿勢をリセット
    pub fn reset(&mut self) {
        self.attitude = Quaternion::identity();
    }

    /// オイラー角を取得（ロール、ピッチ、ヨー）
    #[must_use]
    pub fn euler_angles(&self) -> (f64, f64, f64) {
        self.attitude.to_euler()
    }
}
