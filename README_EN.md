# Adaptive NMPC for Aerial Delivery Without Payload-Event Signals

**Moving Horizon Estimation + NMPC for a quadrotor that grasps and releases
payloads in flight, using only motor speeds and odometry and no external payload-event signal.**

When a delivery drone picks up or drops a parcel, its mass, center of mass, and inertia all
change at once. This project estimates those parameters *online* and feeds them to a
predictive controller, with no force/torque sensor, no external "payload attached" signal,
and no ground-truth measurement of where the payload sits.

![architecture](paper/figs/fig1_architecture.png)

## Results

| | |
|---|---|
| Eventless release flights (payload physically attached, vehicle reached the release command) | **32 / 36** completed; 3.46 s in-maneuver / 16.56 s brake-to-hover median confirmation delay. All four failures were at 4 m/s and trace to estimator failure |
| Paired command-triggered baseline | **19 / 19 vs. 19 / 19** completion; retained as an ablation, not as the proposed method |
| Delayed physical detachment (4 s) | clearing on the command: early in **5 / 5**; persistent confirmation, with or without the command: **0 / 5, 0 / 6** |
| Unsignaled payload loss (no release command reaches the controller) | eventless interface detected **10 / 18**; a command-armed confirmer cannot, by construction (**0 / 18**) |
| Pre-command false releases | **0** in 209 loaded flights (3.7 h; one-sided 95% upper bound 1.4% per flight) |
| Unsignaled partial loss (3 of 4 boxes dropped in flight) | mass estimate followed **14 / 15** steps at 2 m/s |
| vs. L1-adaptive NMPC baseline | comparable transient tracking, **8–32% faster recovery**, plus interpretable physical parameters |
| NMPC solve time @ 20 Hz | 1.3 ms median / 2.2 ms p99 (**4.4% of the control period**) |
| Divergence across the 60-run baseline matrix | **0 / 60** |

![result](paper/figs/fig_mission.png)

The estimator continuously publishes payload mass, first mass moment, and inertia in one
health-gated frame. The payload's CoM offset is recovered from motor-speed-derived torques,
deliberately decoupled from the mass estimate to avoid a bootstrap loop. Release completion
depends on fresh estimator evidence rather than the controller's release command.

## Run it

```bash
# One gripper mission: approach → grasp → figure-eight → phase-triggered release
bash src/scripts/gripper/run_gripper_headless.sh

# Rebuild every figure and table in the paper from frozen logs
bash paper/reproduce.sh
```

Requires ROS 2 Jazzy, PX4-Autopilot (SITL), Gazebo Harmonic, and
[acados](https://github.com/acados/acados). Every number in the paper traces to a
timestamped log — see [REPRODUCE.md](paper/REPRODUCE.md) for the figure-by-figure map.

## How it works

- **MHE** (`offboard_test_acados/mhe_node.py`) — 16-state, 2.0 s window, 10 Hz. Estimates
  payload mass; reconstructs thrust `T_phys` and body torques `τ_phys` from rotor speeds
  alone, which keeps the residual independent of the estimator's own state.
- **Lifecycle evidence** — combines healthy mass, first-moment, inertia, and bounded residual
  evidence; ambiguous releases remain `UNRESOLVED` rather than clearing the payload model.
- **Re-initialization** — after repeated solver failures the MHE re-seeds mass from a window
  vertical-force balance of rotor thrust and odometry (default since 2026-09-15; reproduce
  earlier batches with `MHE_SEED_FROM_THRUST=0 MHE_SEED_WINDOW_BALANCE=0`).
- **NMPC** (`acados_nmpc_node.py`) — acados SQP-RTI, 13 states / 4 inputs, N=20 at
  dt=0.05 s (1.0 s preview), 20 Hz. Consumes `[m, ΔJ, c_xy]` from the estimator.
- **Gazebo plugin** (`gz_plugins/magnetic_gripper/`) — a magnetic DetachableJoint gripper
  that produces genuine mass/CoM/inertia changes. The retired mass-step experiment stack
  is no longer a supported entry point.

## Status

SITL-validated; hardware campaign planned. The 0.30 kg / 4 m/s condition is a stress test at the
vehicle's torque limit (pitch torque saturated in 98% of diagnostic windows), not an operating
point; release on a failed estimator is not yet health-gated. Paper under preparation (target: ICRA 2027).
Chinese technical documentation: [README.md](README.md).
