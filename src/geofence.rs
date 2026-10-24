//! Geofence (`GeofenceShape` / `Geofence` / `GeofenceStatus`).

use crate::vec3::Vec3;

// ジオフェンス
// ============================================================

/// ジオフェンスの形状
#[derive(Debug, Clone)]
pub enum GeofenceShape {
    /// 円柱型ジオフェンス（中心、半径、最小高度、最大高度）
    Cylinder {
        center: Vec3,
        radius: f64,
        min_altitude: f64,
        max_altitude: f64,
    },
    /// 矩形型ジオフェンス（最小座標、最大座標）
    Box { min: Vec3, max: Vec3 },
}

/// ジオフェンスチェッカー
#[derive(Debug, Clone)]
pub struct Geofence {
    /// ジオフェンス領域
    shape: GeofenceShape,
    /// 警告マージン（境界からの距離）
    warning_margin: f64,
}

/// ジオフェンスのチェック結果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeofenceStatus {
    /// 安全領域内
    Inside,
    /// 警告領域（境界付近）
    Warning,
    /// 境界外（違反）
    Outside,
}

impl Geofence {
    /// 円柱型ジオフェンスを生成
    #[must_use]
    pub fn cylinder(
        center: Vec3,
        radius: f64,
        min_altitude: f64,
        max_altitude: f64,
        warning_margin: f64,
    ) -> Self {
        Self {
            shape: GeofenceShape::Cylinder {
                center,
                radius,
                min_altitude,
                max_altitude,
            },
            warning_margin: warning_margin.max(0.0),
        }
    }

    /// 矩形型ジオフェンスを生成
    #[must_use]
    pub fn bounding_box(min: Vec3, max: Vec3, warning_margin: f64) -> Self {
        Self {
            shape: GeofenceShape::Box { min, max },
            warning_margin: warning_margin.max(0.0),
        }
    }

    /// 位置がジオフェンス内かチェック
    #[must_use]
    pub fn check(&self, pos: Vec3) -> GeofenceStatus {
        match &self.shape {
            GeofenceShape::Cylinder {
                center,
                radius,
                min_altitude,
                max_altitude,
            } => {
                let horiz_dist = pos.horizontal_distance_to(*center);
                let alt = pos.y;

                // 完全に外
                if horiz_dist > *radius || alt < *min_altitude || alt > *max_altitude {
                    return GeofenceStatus::Outside;
                }
                // 警告領域
                if horiz_dist > radius - self.warning_margin
                    || alt < min_altitude + self.warning_margin
                    || alt > max_altitude - self.warning_margin
                {
                    return GeofenceStatus::Warning;
                }
                GeofenceStatus::Inside
            }
            GeofenceShape::Box { min, max } => {
                // 完全に外
                if pos.x < min.x
                    || pos.x > max.x
                    || pos.y < min.y
                    || pos.y > max.y
                    || pos.z < min.z
                    || pos.z > max.z
                {
                    return GeofenceStatus::Outside;
                }
                // 警告領域
                let m = self.warning_margin;
                if pos.x < min.x + m
                    || pos.x > max.x - m
                    || pos.y < min.y + m
                    || pos.y > max.y - m
                    || pos.z < min.z + m
                    || pos.z > max.z - m
                {
                    return GeofenceStatus::Warning;
                }
                GeofenceStatus::Inside
            }
        }
    }

    /// 境界までの最短距離（負=外側）
    #[must_use]
    pub fn distance_to_boundary(&self, pos: Vec3) -> f64 {
        match &self.shape {
            GeofenceShape::Cylinder {
                center,
                radius,
                min_altitude,
                max_altitude,
            } => {
                let horiz_dist = pos.horizontal_distance_to(*center);
                let radial = radius - horiz_dist;
                let alt_low = pos.y - min_altitude;
                let alt_high = max_altitude - pos.y;
                radial.min(alt_low).min(alt_high)
            }
            GeofenceShape::Box { min, max } => {
                let dx_min = pos.x - min.x;
                let dx_max = max.x - pos.x;
                let dy_min = pos.y - min.y;
                let dy_max = max.y - pos.y;
                let dz_min = pos.z - min.z;
                let dz_max = max.z - pos.z;
                dx_min
                    .min(dx_max)
                    .min(dy_min)
                    .min(dy_max)
                    .min(dz_min)
                    .min(dz_max)
            }
        }
    }
}
