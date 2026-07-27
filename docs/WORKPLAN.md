# Workplan

Twelve phases. The estimates assume steady part-time work; the sequencing matters more
than the numbers. Every phase has an explicit done-when, because "working on the nav
stack" is how projects die.

Progress is tracked here rather than in an issue tracker so that the plan and the code
move in the same commit.

---

## Phase 0 — Environment · ✅ done

Monorepo laid out as `rust/ cpp/ java/ python/ sim/ infra/ docs/`. ROS 2 Jazzy and
Gazebo run in Docker, so the host OS is irrelevant and CI runs the same image a
developer does.

**Done when:** `ros2 launch depot_sim warehouse.launch.py` gives a drivable robot and
RViz shows live lidar.

> Partially met. The repository layout, the Rust toolchain, and the container build
> exist; the Gazebo world and the launch file do not yet. Tracked as the remainder of
> Phase 0 alongside Phase 2.

## Phase 1 — Rust safety layer · 🟡 in progress

Built first, before navigation, because it is the architectural spine. Building it
first forces the interface to be real rather than retrofitted.

- [x] `depot-safety-core`: `no_std`, no allocation, no I/O, no clock reads. Field
      geometry, speed-scaled zones, the state machine, the arbitration function.
- [x] Property tests: output always within vehicle limits; protective stops immediate;
      acceleration limits respected; zone limits can only restrict; e-stop latches;
      bit-exact determinism.
- [x] Allocation guard: a counting global allocator proves zero heap traffic across
      every branch.
- [x] Latency budget asserted in test. Release build, 1081 rays: p50 4.6 µs, p99 8.5 µs
      against a 10 ms budget.
- [ ] `depot-safety-node`: ROS 2 binding via `r2r`. Subscribes lidar and `cmd_vel_raw`,
      publishes `cmd_vel`.

**Done when:** you teleop the robot directly at a wall at full speed and it stops
itself; then you kill the teleop node mid-drive and it ramps to zero.

> Both behaviours are proven in simulation-free unit form
> (`tests/arbitration.rs`). The claim is only fully discharged once the ROS 2 node
> exists and the same two things happen in Gazebo.

## Phase 2 — C++ navigation, single robot

Sensor pipeline, `robot_localization` EKF, `slam_toolbox` mapping, AMCL, Nav2 for a
diff-drive AMR publishing to `cmd_vel_raw` so everything passes through the Rust layer.

**Done when:** a goal clicked in RViz gets driven to, and a surprise obstacle triggers
the safety layer rather than only Nav2's own avoidance.

## Phase 3 — Python simulation harness

Test infrastructure before the system needs it. This phase feels premature and is not.
Scenario schema, headless launcher, metrics, CI suite.

**Done when:** `python -m depot_lab.run --scenario basic_nav --seed 0..20` produces a
results table and CI fails on a nonzero collision count.

## Phase 4 — Java fleet service, single robot

Spring Boot, domain model, robot registry over a ROS 2 bridge, greedy assignment,
telemetry persistence, basic dashboard.

**Done when:** POSTing a job drives the robot to the pickup point and reports
completion, with the mission visible in the dashboard.

## Phase 5 — Shelf docking

Fiducials, 6-DoF pose estimation, a visual-servoing controller distinct from Nav2 path
following, simulated lift. The Rust layer gets a docking mode with a modified
protective field — explicitly and narrowly, never by disabling safety.

**Done when:** approach, align within ±2 cm / ±3°, lift, transport, and drop, ten times
consecutively.

## Phase 6 — Multi-robot, naive

Four robots, no coordination. Run it and watch what breaks.

**Done when:** failure modes are documented and captured as regression scenarios —
head-on standoffs, intersection near-misses, livelock at doorways.

## Phase 7 — Java traffic management

The most valuable phase; budget generously. Roadmap graph, time-windowed reservations,
wait-for-graph deadlock detection, priority preemption, constraints fed back into Nav2.

**Done when:** 8 robots run a 30-minute mixed workload with zero collisions, zero
deadlocks, and no protective stops caused by robot-robot interaction. Protective stops
should be *rare*. If the Rust layer is doing the collision avoidance, traffic
management has failed.

## Phase 8 — Task allocation

Greedy → Hungarian → auction protocol tolerating joins and dropouts.

**Done when:** a chart shows throughput against allocation strategy and fleet size, with
an explanation of where each strategy wins.

## Phase 9 — Battery and charging

Drain model, chargers as a reservable resource, predictive routing.

**Done when:** an 8-hour simulated shift completes with zero stranded robots and
charger utilisation reported.

## Phase 10 — Python perception training

Synthetic labelled data from Gazebo, PyTorch detectors, ONNX export, C++ inference.
Compare against the classical baseline honestly — if classical wins on fiducials, say
so, because it often does.

**Done when:** human detection drives speed-zone constraints into the Rust layer and a
person walking toward a robot makes it slow before it stops.

## Phase 11 — Analytics and hardening

Congestion heatmaps, deadlock hotspots, saturation point. Chaos testing: kill nodes,
drop sensors, partition the network.

**Done when:** every chaos scenario ends with robots safely stopped and the fleet
recovers when the fault clears.

## Phase 12 — Presentation

Architecture write-up, a recording of 8 robots running a shift, and two deep dives:
the safety arbitration design and the deadlock resolution algorithm. Publish the
scaling numbers, including where it falls over and why.

## Optional Phase 13 — Hardware

Expect the Rust layer to port cleanly, the perception to need recalibration, and
everything about timing to get worse.

---

## Cut-down version

If the full plan is too long, the minimum that still demonstrates the architecture is
phases 0, 1, 2, 3, 4, 6, 7. The safety arbitration boundary and the deadlock work —
the two genuinely distinctive pieces — both survive the cut.

## Standing risks

**Nav2 tuning is a time sink.** Timebox it. The project's value is not in a perfectly
tuned local planner.

**Multi-robot simulation is expensive.** Reduce lidar rays and camera resolution for
scenario runs; keep full fidelity for demos only.

**The ROS 2 / Java bridge is the weakest joint.** There is no first-class Java ROS 2
client. Decide in Phase 4 and do not relitigate it.

**The safety layer will tempt you to weaken it.** Every inconvenient stop makes
shrinking the field look attractive. If it fires often, the layer above is planning
badly. Fix the cause.
