# Adaptive NMPC for Aerial Delivery Without Payload-Event Signals

**Self-triggered Moving Horizon Estimation + NMPC for a quadrotor that grasps and releases
payloads in flight, using only motor speeds and odometry and no external payload-event signal.**

When a delivery drone picks up or drops a parcel, its mass, center of mass, and inertia all
change at once. This project estimates those parameters *online* and feeds them to a
predictive controller, with no force/torque sensor, no external "payload attached" signal,
and no ground-truth measurement of where the payload sits.

![architecture](../paper/figs/fig1_architecture.png)

## Results

| | |
|---|---|
| Mass-estimate settling after a payload event | **4.0× faster** (1.20 s → 0.30 s) vs. fixed-weight MHE |
| Same, with **no external event signal** (self-triggered on thrust residuals) | **within 0.02 s** of the signal-fed version |
| vs. L1-adaptive NMPC baseline | equal transient tracking, **15–35% faster recovery**, plus interpretable physical parameters |
| NMPC solve time @ 20 Hz | 1.3 ms median / 2.2 ms p99 (**4.4% of the control period**) |
| Divergence across the 60-run baseline matrix | **0 / 60** |

![result](../paper/figs/fig3_mest_timeline.png)

The estimator deweights the pre-event stages of its sliding window the moment a thrust
residual crosses a threshold, so stale measurements from before the discontinuity stop
dragging the estimate. The payload's CoM offset is recovered from motor-speed-derived
torques, deliberately decoupled from the mass estimate to avoid a bootstrap loop.

**We also report a negative result.** We embedded the hand-designed weight schedule in a
5-parameter family and optimized it by simulation-in-the-loop cross-entropy search. Under a
pre-registered paired factorial ablation (n=8/cell, Latin-square order rotation, two solver
rates, 97% power) it produced **no measurable gain**. The mechanism is informative: the
objective is flat over a broad neighbourhood of the hand-designed rule, so on this task
*when* to reweight carries the benefit and *how much* is weakly determined. Details in
[the paper](../paper/main.pdf) §VI-B.

## Run it

```bash
# One gripper mission: approach → grasp → figure-eight → phase-triggered release
bash src/scripts/gripper/run_gripper_headless.sh

# Rebuild every figure and table in the paper from frozen logs
bash paper/reproduce.sh
```

Requires ROS 2 Jazzy, PX4-Autopilot (SITL), Gazebo Harmonic, and
[acados](https://github.com/acados/acados). Every number in the paper traces to a
timestamped log — see [REPRODUCE.md](../paper/REPRODUCE.md) for the figure-by-figure map.

## How it works

- **MHE** (`offboard_test_acados/mhe_node.py`) — 20-state, 2.0 s window, 10 Hz. Estimates
  payload mass; reconstructs thrust `T_phys` and body torques `τ_phys` from rotor speeds
  alone, which keeps the residual independent of the estimator's own state.
- **Event trigger** (`mhe_event_weights.py`) — self-detects payload events from the thrust
  residual with a warmup gate (rejects the takeoff transient) and a persistence count
  (rejects gusts), then reschedules the MHE stage weights.
- **NMPC** (`acados_nmpc_node.py`) — acados SQP-RTI, 13 states / 4 inputs, N=20 at
  dt=0.05 s (1.0 s preview), 20 Hz. Consumes `[m, ΔJ, c_xy]` from the estimator.
- **Gazebo plugin** (`gz_plugins/magnetic_gripper/`) — a magnetic DetachableJoint gripper
  that produces genuine mass/CoM/inertia changes. The retired mass-step experiment stack
  is no longer a supported entry point.

## Status

SITL-validated; hardware campaign planned. Paper under preparation (target: ICRA 2027).
Chinese technical documentation: [README.md](README.md).
