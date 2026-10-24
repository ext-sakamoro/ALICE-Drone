[English](README.md) | **日本語**

# ALICE-Drone

ALICEエコシステムのドローン/UAV飛行制御ライブラリ。PID制御、ウェイポイントナビゲーション、姿勢推定、ジオフェンス、バッテリー管理、フライトモードステートマシンを純Rustで実装。

## 概要

| 項目 | 値 |
|------|-----|
| **クレート名** | `alice-drone` |
| **バージョン** | 1.0.0 |
| **ライセンス** | AGPL-3.0 |
| **エディション** | 2021 |

## 機能

- **3Dベクトル演算** — 内積・外積・正規化・距離・水平距離を持つ `Vec3`
- **PID制御** — 比例・積分・微分ゲイン設定可能、積分ワインドアップ制限付き
- **ウェイポイントナビゲーション** — 到着半径検出による順次ウェイポイント追従
- **姿勢推定** — クォータニオンベースの姿勢追跡
- **ジオフェンス** — 円筒形境界エンフォースメント（半径＋高度制限）
- **バッテリー管理** — 電圧・容量監視とRTH (Return-to-Home) 閾値
- **フライトモード** — 状態機械: Idle / Takeoff / Hover / Waypoint / ReturnToHome / Land / Emergency

## アーキテクチャ

```
alice-drone (lib.rs — 単一ファイルクレート)
├── Vec3                        # 3Dベクトル演算
├── PidGains / PidController    # PID制御ループ
├── Waypoint / WaypointNav      # ナビゲーションエンジン
├── Attitude                    # 姿勢追跡
├── Geofence                    # 境界制御
├── Battery                     # 電力管理
├── FlightMode / FlightState    # モード状態機械
└── Drone                       # トップレベルUAVコントローラー
```

## クイックスタート

```rust
use alice_drone::{Drone, Vec3, Waypoint};

let mut drone = Drone::new();
drone.add_waypoint(Waypoint::new(Vec3::new(10.0, 5.0, 0.0), 1.0));
drone.takeoff(5.0);
drone.update(0.01); // 100 Hz でシミュレーション更新
```

## ビルド

```bash
cargo build
cargo test
cargo clippy -- -W clippy::all
```

## ライセンス

`LicenseRef-Proprietary` 条文は [LICENSE-Proprietary](LICENSE-Proprietary)

Enquiries: https://extoria.co.jp/
