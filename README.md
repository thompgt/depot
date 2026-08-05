# Depot — Warehouse AMR Fleet

A four-language autonomous mobile robot system. Robots drive beneath mobile shelf
units, lift them, and deliver them to picking stations while sharing corridors with
each other and with humans. Built and validated entirely in simulation.

## Tech Stack

![Rust](https://img.shields.io/badge/Rust-000000?style=for-the-badge&logo=rust&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Java](https://img.shields.io/badge/Java-ED8B00?style=for-the-badge&logo=openjdk&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-2496ED?style=for-the-badge&logo=docker&logoColor=white)

| Language | Package | Owns |
|---|---|---|
| Rust | `depot-safety` | Protective fields, velocity arbitration, watchdog |
| C++ | `depot-nav` | Perception, SLAM, localization, Nav2, docking |
| Java | `depot-fleet` | Task allocation, traffic management, battery, telemetry |
| Python | `depot-lab` | Training, scenario harness, analytics, calibration |

---

## Architecture

```
                    ┌─────────────────────────────────────┐
                    │  Java: depot-fleet                  │
                    │  tasks · traffic · battery · replay  │
                    └──────────────┬──────────────────────┘
                                   │  goals / reservations (seconds)
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  C++: depot-nav                     │
                    │  SLAM · Nav2 · perception · docking  │
                    └──────────────┬──────────────────────┘
                                   │  cmd_vel (30–100ms)
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  Rust: depot-safety     ◄── lidar    │
                    │  fields · arbitration · watchdog     │
                    └──────────────┬──────────────────────┘
                                   │  arbitrated cmd_vel (<10ms)
                                   ▼
                              motors / Gazebo
```

**Control authority decreases as you go up.** The fleet service *requests*, the nav
stack *plans*, and the safety layer *decides*. Anything above Rust can be wrong
without being dangerous.

### Latency budgets

| Component | Budget | Failure mode if missed |
|---|---|---|
| `depot-safety` | 10 ms hard | Collision |
| `depot-nav` local planner | 100 ms soft | Jerky motion, missed obstacles |
| `depot-fleet` allocation | 1 s soft | Idle robots, poor throughput |
| `depot-lab` | offline | None |

A latency budget you don't measure is a wish, so these are asserted in tests.

Measured for `depot-safety-core` at its worst-case 1081-ray scan, release build:

| | mean | p50 | p99 | budget |
|---|---|---|---|---|
| arbitration cycle | 5.1 µs | 4.6 µs | 8.5 µs | 10,000 µs |

Three orders of magnitude of headroom, which is the point: the arbitration algorithm
should be irrelevant to the deadline, leaving the entire budget to scheduling, transport
and the operating system — the parts that are actually hard to control. The observed
*maximum* on a desktop OS is milliseconds and is pure scheduler preemption; the test
prints it rather than asserting on it, and eliminating it is a real-time configuration
problem rather than an algorithmic one.

---

## Design position: Python never enters a control loop

`depot-lab` produces **artifacts** — ONNX weights, calibration files, tuned
parameters, scenario definitions. It never subscribes to a topic with a deadline
attached, and no Python process sits between a sensor and a motor.

This is a deliberate constraint, not an accident of packaging. CPython's garbage
collector and global interpreter lock make *tail* latency unpredictable in a way
that mean latency hides: a p99.9 pause of tens of milliseconds is unremarkable for a
training job and is a collision in a 10 ms arbitration loop. Python is excellent at
the offline half of robotics — learning, analysis, test generation — and that is
exactly where it lives here.

The boundary is enforced by the artifact format. C++ loads `.onnx` files; it does
not call Python. Python reads telemetry; it does not write commands.

---

## Repository layout

```
rust/     depot-safety-core (no_std) and the ROS 2 node binding
cpp/      ROS 2 packages: perception, localization, navigation, docking
java/     Spring Boot fleet service
python/   depot-lab: training, scenario harness, analytics
sim/      Gazebo worlds, robot models, launch files
infra/    Dockerfiles, compose, CI configuration
docs/     Design notes and architecture decision records
```

---

## Status

Under construction. See [`docs/WORKPLAN.md`](docs/WORKPLAN.md) for the phase plan and
the done-when criteria for each.

- [x] Phase 0 — environment scaffolding
- [ ] Phase 1 — Rust safety layer *(core done and proven; ROS 2 node outstanding)*
- [ ] Phase 2 — C++ navigation, single robot
- [ ] Phase 3 — Python simulation harness
- [ ] Phase 4 — Java fleet service, single robot

---

## Building

The simulation stack targets ROS 2 Jazzy on Ubuntu 24.04 and runs in Docker, so the
host OS does not matter:

```bash
docker compose -f infra/compose.yaml build
docker compose -f infra/compose.yaml run --rm dev
```

The Rust safety core has no ROS dependency and builds anywhere:

```bash
cd rust
cargo test --all-targets                       # 47 tests: unit, behavioural, property
cargo test --release --test latency -- --nocapture   # the 10 ms budget, measured
cargo test --release --test no_alloc           # the zero-allocation guarantee
cargo build -p depot-safety-core --target thumbv7em-none-eabihf   # still bare-metal
```

That last line is a standing check rather than a demonstration. The core is `no_std` so
that the identical arbitration logic could run on a safety MCU beside the motor
controller, and the surest way to lose that property is to let one convenient `String`
in an error path go unnoticed. CI cross-compiles it on every push.
