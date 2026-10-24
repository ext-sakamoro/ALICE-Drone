**English** | [日本語](README_JP.md)

# ALICE-Drone

Drone / UAV flight control library for the ALICE ecosystem. Pure Rust implementation covering PID control, waypoint navigation, attitude estimation, geofencing, battery management, and flight mode state machine.

## Overview

| Item | Value |
|------|-------|
| **Crate** | `alice-drone` |
| **Version** | 1.0.0 |
| **License** | AGPL-3.0 |
| **Edition** | 2021 |

## Features

- **3D Vector Math** — `Vec3` with dot/cross product, normalization, distance, and horizontal distance
- **PID Controller** — Configurable proportional, integral, and derivative gains with integral windup clamping
- **Waypoint Navigation** — Sequential waypoint following with arrival radius detection
- **Attitude Estimation** — Quaternion-based orientation tracking
- **Geofence** — Cylindrical boundary enforcement (radius + altitude limits)
- **Battery Management** — Voltage/capacity monitoring with return-to-home threshold
- **Flight Modes** — State machine: Idle, Takeoff, Hover, Waypoint, ReturnToHome, Land, Emergency

## Architecture

```
alice-drone (lib.rs — single-file crate)
├── Vec3                        # 3D vector operations
├── PidGains / PidController    # PID control loop
├── Waypoint / WaypointNav      # Navigation engine
├── Attitude                    # Orientation tracking
├── Geofence                    # Boundary enforcement
├── Battery                     # Power management
├── FlightMode / FlightState    # Mode state machine
└── Drone                       # Top-level UAV controller
```

## Quick Start

```rust
use alice_drone::{Drone, Vec3, Waypoint};

let mut drone = Drone::new();
drone.add_waypoint(Waypoint::new(Vec3::new(10.0, 5.0, 0.0), 1.0));
drone.takeoff(5.0);
drone.update(0.01); // step simulation at 100 Hz
```

## Build

```bash
cargo build
cargo test
cargo clippy -- -W clippy::all
```

## License

`LicenseRef-Proprietary`. The text is [LICENSE-Proprietary](LICENSE-Proprietary).

Enquiries: https://extoria.co.jp/
