# Signal-Free Payload-Aware MHE + NMPC: Grasp, Maneuver, Release

> For readers who want to understand the method quickly (advisor, lab members, newcomers). Assumes
> familiarity with quadrotors and the basics of MPC/MHE.
> This note covers the theory and data flow only, not code details. It describes the default
> configuration as of 2026-09-17.
> Chinese version: [算法简介_无信号载荷感知MHE-NMPC.md](算法简介_无信号载荷感知MHE-NMPC.md).

## 1. The problem

A quadrotor picks up a box whose **mass and offset are both unknown**, flies an aggressive figure-8
with it (about 4 m/s), and then drops it.

The difficulty is that **neither the controller nor the estimator knows anything in advance**:

- it does not know how heavy the box is or how far off-center it hangs;
- it receives no "attached" or "released" notification.

When the payload changes, three things in the drone's dynamics change at once:

| Change | Consequence |
| --- | --- |
| Total mass increases | Hover thrust is wrong; the drone sinks or overshoots |
| Center of mass shifts | Thrust no longer passes through the CoM, creating a constant twisting torque |
| Inertia increases | The same torque turns the drone more slowly; attitude response becomes sluggish |

There is also a safety requirement: **sending a release command does not mean the box has actually
left.** The gripper may jam, the box may detach late, or it may fall off with no command at all. The
controller must not assume it is empty just because it sent the command.

**Goal of the method**: estimate the payload online from the drone's own measurements alone, so the
controller always flies with the right model, and conclude that "the payload is gone" only when
there is physical evidence.

## 2. Overall idea

The system has two layers, each updated on its own fixed period:

```
 Motor speeds ─┐
               ├─► MHE (estimator, 10 Hz) ──payload estimate──► NMPC (controller, 20 Hz) ──body rates + thrust──► flight controller
 Odometry ─────┘                                                  ▲
                                                                  └── Odometry (current state)
```

- **MHE** (moving horizon estimation): every 0.1 s it looks over the last 2 seconds of flight data
  and works out "what the payload is".
- **NMPC** (model predictive control): every 0.05 s it updates its model with the latest payload
  estimate, predicts 1 second ahead, and computes the optimal control input.
- **Information flows one way only**, from MHE to NMPC. The MHE never looks at the controller's
  commands and receives no attach or release events.

## 3. Measurements: getting the "actual forces" from motor speeds

The MHE needs the thrust and torque the drone actually experiences at every instant. These are
**computed directly from motor speeds**, not taken from the controller's commands:

- thrust of each rotor `F_i = k · ω_i²`, total thrust `T = Σ F_i`;
- the three torques follow from each rotor's thrust times its lever arm (plus the rotor drag
  coefficient for yaw).

**Why not use the controller's command?** The command itself is computed from the current mass
estimate. If the mass is wrong, the command is wrong too, and the estimator would use a wrong input
to "confirm" a wrong estimate, never noticing the problem. Motor speed is what the actuators actually
produced and is independent of the estimate, so it genuinely tests whether the estimate is right.

**The forces must also be interval-averaged.** Motor data arrive at 250 Hz, but the MHE samples only
every 0.1 s. Taking just the latest sample at each tick (point sampling) aliases badly while the drone
oscillates, and can be far from the force actually applied over those 0.1 s. So each step uses the
**average thrust and torque** over all motor samples in that 0.1 s.

## 4. Describing the payload: mass + first mass moment

The payload is described by just two quantities:

- **total mass m** (drone plus payload);
- **first mass moment s = m_P · r_xy**: payload mass times its horizontal offset (2-D).

Everything else about the payload follows algebraically from these two:

Treat the payload as a point mass at position `r = [r_x, r_y, r_z]` relative to the drone's own CoM
(r_xy is the horizontal part, r_z the vertical part), and let m_B be the empty mass:

```
payload mass                  m_P  = m − m_B
horizontal composite-CoM      c_xy = m_P · r_xy / m = s / m        (relative to the drone's CoM)
reduced mass                  μ    = m_B · m_P / m
inertia increment             ΔJ   = μ · (|r|² I₃ − r rᵀ)          (about the composite CoM, exact form)
```

The method keeps only the leading and coupling terms of ΔJ:

```
ΔJ_xx ≈ ΔJ_yy ≈ μ · r_z²                                 (drops μ·r_y², μ·r_x², about 2%)
ΔJ_xz ≈ −μ · r_x · r_z = −(m_B / m) · s_x · r_z
ΔJ_yz ≈ −μ · r_y · r_z = −(m_B / m) · s_y · r_z
ΔJ_zz ≈ 0                                                 (drops μ·|r_xy|²)
```

- The vertical offset r_z is given by the gripper geometry and treated as known; it is not estimated
  online (it is not observable from thrust–torque data in the first place).
- Every retained term needs only m and s, never m_P and r_xy separately; the payload's own rotational
  inertia is neglected.
- The MHE model uses both the diagonal and coupling terms above; the NMPC model uses only the diagonal
  terms ΔJ_xx and ΔJ_yy.

**Why estimate s instead of the offset r_xy directly?**

- Once the box is gone, `m_P → 0` and the offset `r_xy` becomes 0/0, which is undefined; but
  `s = m_P · r_xy` naturally goes to 0 and is always defined.
- In flight data, payload mass and offset **cannot be separated to begin with**; only their product
  is visible (the offset torque is proportional to m_P · r_xy). Estimating s estimates exactly the
  observable part.
- With an offset, rotational data carry a lot of information about mass (about 94% of the mass
  information comes through rotation), so including s in the model also makes the mass estimate more
  accurate.

## 5. MHE: how the payload estimate is updated each period

### 5.1 Sliding window

The MHE always keeps a window of the latest **20 steps, 2 seconds** of data. Each period it does four
things:

1. adds the newest measurement (position, velocity, attitude, body rate) and the interval-averaged
   thrust and torque to the window;
2. drops the oldest step;
3. solves an optimization problem over the whole window;
4. takes m and s at the **end of the window (the current time)** as this period's estimate.

Because the window slides, the estimate **updates continuously, frame by frame**: when the payload
changes, new data come in one frame at a time and the estimate climbs with them, without waiting for
the window to fill.

### 5.2 The optimization problem

Within the window, the drone state plus m and s are all unknowns, and the solver looks for the values
that best explain the 2 seconds of observations:

```
min   arrival cost (deviation of the window start from the previous estimate)
    + Σ measurement residual² (model-predicted state − measured state)
    + Σ process noise² (small deviations allowed between model and true dynamics)

s.t.  rigid-body dynamics (thrust and torque are known inputs; mass, CoM offset and inertia are generated from m and s)
      m and s constant within the window
      m ≥ 0.95 × empty mass (physical lower bound)
```

Key design choices:

**① Measurement-noise weights** cover all 13 measured states (mass and first moment are not measured
directly, and thrust and torque are known inputs, so none of them appear in the measurement residual).
Each weight is `1/σ²`, where σ is an order-of-magnitude assumption based on the typical accuracy of the
fused state estimate, not a measured value.

| Measured quantity | Dim. | Noise std σ | Weight 1/σ² |
| --- | --- | --- | --- |
| Position | 3 | 0.02 m | 2500 |
| Velocity | 3 | 0.05 m/s | 400 |
| Attitude quaternion | 4 | 0.01 (about 1.1°) | 10⁴ |
| Body rate | 3 | 0.02 rad/s | 2500 |

**② Process-noise weights** act only on the 13 physical states. m and s are constant within the window
and are not driven by process noise (parameter random walk is off by default).

| State | Dim. | Weight | Equivalent σ = 1/√weight |
| --- | --- | --- | --- |
| Position | 3 | 10⁴ | 0.01 m |
| Velocity | 3 | 10⁴ | 0.01 m/s |
| Attitude quaternion | 4 | 10⁵ | 0.0032 |
| Body rate | 3 | 10³ | 0.032 rad/s |

- Process noise is **tighter** than measurement noise overall (except for body rate). This says "apart
  from the unknown payload, the model structure is right": the optimizer can explain the observed
  accelerations and angular accelerations only by adjusting m and s, rather than pushing the error onto
  process noise.
- Velocity is the tightest (25× its measurement weight), because mass acts on velocity mainly through
  `T/m`.
- Body rate is deliberately looser, because the gyroscopic coupling term in the rotational dynamics is
  numerically more sensitive, and the slack avoids ill-conditioning.

**③ Arrival-cost weights** span 16 dimensions (13 physical states + m + the 2 components of s). Each
period, the prior mean is the state at step 2 of the previous window's solution — the new starting point
after the window shifts by one step — which keeps the estimate consistent over time.

| State | Dim. | Weight | Equivalent σ |
| --- | --- | --- | --- |
| Position | 3 | 10³ | 0.032 m |
| Velocity | 3 | 10³ | 0.032 m/s |
| Attitude quaternion | 4 | 10⁴ | 0.01 |
| Body rate | 3 | 10² | 0.1 rad/s |
| Mass m | 1 | 0.1 | about 3.2 kg |
| First moment s | 2 | 100 | 0.1 kg·m |

- Physical states are weighted an order of magnitude below the process noise: a moderate anchor only.
- Mass gets almost no prior, so a mass step from a grasp or release re-converges within about one
  window, and the estimate does not become an echo of the prior. (It is not set to exactly 0, to keep the
  problem numerically positive definite when excitation is weak.)
- The first moment gets only a very weak anchor. Tightening it suppresses over-estimation of s with small
  payloads; loosening it lets s return to zero faster after release. The value is a compromise between the
  two.

**④ The solver's initial guess comes from a vertical force balance over the window**, not from the
empty mass:

```
m_seed = Σ_j T_j·cosθ_j / [ N·(g + ā_z) ]        j runs over the N steps in the window

cosθ_j: projection of the thrust direction onto the world vertical (from the attitude quaternion)
ā_z   : mean vertical acceleration over the window (velocity difference between the two ends divided by the window length)
```

In words: the mean vertical thrust component over the window, divided by (gravity + mean vertical
acceleration). Using the whole window rather than a single frame averages out noise and maneuver
fluctuations. Several guards apply: the window must be full, mean thrust at least 1 N (i.e. actually
flying), a sane quaternion, and a denominator above `0.5·N·g` (which rejects near-free-fall windows).
If they fail it falls back to the instantaneous `T/g`, and only then to the empty mass with a warning.

⚠️ This value enters only the **initial guess** and the arrival-cost prior mean, and the arrival weight on
mass is just 0.1, so it barely enters the cost at all: it affects how fast the solve converges, not the
optimum itself — it is an initial guess, not a prior. The first moment s always starts from 0.

### 5.3 Recovering from errors (re-anchoring)

If the estimator falls into a wrong solution, it re-seeds m from the thrust balance, resets s to zero
and starts over. There are two triggers:

1. 5 consecutive solver failures;
2. in calm flight, s stays above the largest physically possible value (maximum payload × maximum
   offset) for 1 second — meaning the solve "succeeded" but the result is physically impossible.

### 5.4 What is output each period

Each period, the MHE **packs the following into one frame** for the NMPC. All values in a frame come
from the same solve, so combinations that cannot exist physically, such as a new mass with an old
inertia, never occur:

- mass m, first moment s, and the CoM offset c and inertia increment ΔJ derived from them;
- **empty-payload confidence** (0–1): combines evidence from mass, first moment and inertia to express
  how credible it is that "there is no payload now";
- **health flag** and **solution age**: whether this frame is trustworthy, and how long since the last
  successful solve.

### 5.5 Speeding up convergence after a payload step (event de-weighting)

**Why it is needed**: m and s are constant within the window. Right after the payload changes, the
window holds data from before and after the change at the same time; the two pull against each other
and the optimizer can only return a compromise, converging fully only once the old data have slid out
of the window (2 seconds).

**How a step is detected (no external signal)**: a slow baseline of the motor-derived thrust T is
maintained (time constant 3 s). The detector arms only after T has stayed close to that baseline for a
while. After that, as soon as T deviates from the baseline by more than a threshold (about 4.4 N =
1.5 × g × payload envelope) for 2 consecutive frames, the **frame where it first crossed** is recorded
as the event frame. T below the baseline means the payload is gone; above means one was picked up.

**How de-weighting works**: while the event frame is still inside the window, the measurement-residual
and process-noise weights of every step **before** the event are scaled by 10⁻⁴, while steps after it
keep their nominal weights. The old data then barely pull on m and s, and the estimate can jump to the
new value within a frame or two. Once the event slides out of the window, the weights recover
automatically.

A few details:

- The weights are **scaled relatively rather than set to zero**: nominal weights are 10³–10⁵, so after
  scaling they are still well-conditioned values of 0.1–10, while being four orders of magnitude below
  the post-event steps — enough for the new data to dominate.
- **The window is not cleared**: the physical states keep their warm start, so there is no need to wait
  for the window to refill.
- **The arrival cost is not de-weighted**: the anchor on the physical states stays as it is, and the
  anchor on mass is only 0.1 anyway, so it is not a bottleneck.

## 6. Payload state: when is it "carrying", and when is it "gone"?

The estimator maintains its own "empty / carrying" state, with no external events at all.

**Carrying** is declared when payload mass stays clearly positive. When the thrust residual jumps, older
data in the window are temporarily down-weighted as described in Section 5.5, so the estimate catches up
faster.

**Released** requires several kinds of evidence; no single one is enough. There are three:

- **First-moment collapse**: once carrying is confirmed, the "|s| while carrying" is recorded as a
  reference during a calm stretch of flight, and **frozen from then on**. If |s| later falls below 30% of
  that reference and stays there, that counts as a vote. The reference must be frozen, or inflated values
  caused by maneuvering would contaminate it.
- **Mass drop**: payload mass stays below 30 g.
- **Thrust residual**: a sudden drop in thrust.

In addition, **no decision is made while the estimate is unhealthy**.

**Why so cautious?** Experiments showed that the thrust residual caused by the figure-8 maneuver itself
can be larger than the one caused by a small payload actually dropping, and that the mass estimate dips
briefly during aggressive maneuvers. Looking at one signal alone cannot tell "maneuvering" from
"actually dropped".

## 7. NMPC: how the payload estimate is used each period

### 7.1 Updating the payload model

Each control period (0.05 s), the NMPC does three things:

1. reads the latest MHE frame. If the frame is unhealthy or has not been updated for more than
   0.35 s, **the last safe model is kept unchanged**;
2. low-pass filters m and s and limits how fast they may change, so estimate jitter does not hit the
   controller directly;
3. computes the CoM offset `c = s/m` and the inertia increment `ΔJ` and writes them into the
   prediction model.

**Inertia fallback at the moment of attaching**: right after the attach command, the MHE has not yet
had time to estimate the payload. The inertia is first given a safe floor based on the largest possible
payload; once the MHE shows sustained evidence of carrying, it is handed over smoothly to the MHE's
inertia estimate within 0.5 s.

### 7.2 How the payload enters the prediction model

```
translation:  v̇ = (R·[0, 0, T] − k_d·v) / m − g
rotation:     ω̇ = (τ + [−c_y·T, +c_x·T, 0] − ω × Jω) / (J + ΔJ)
```

- **Mass m** sets how much acceleration a given thrust produces;
- **CoM offset c** produces the offset torque `c × T`, so the NMPC automatically outputs a trim torque
  that cancels it;
- **Inertia increment ΔJ** tells the model the drone now turns more slowly.

The input reference in the cost function also follows the payload: the reference thrust is `m·g` and
the reference torque is the trim torque above. Otherwise the cost would keep pulling inputs toward
empty-drone hover values and cause steady-state bias.

### 7.3 Solving and actuation

- One solve per period: predict 1 second ahead (20 steps) and find the optimal inputs within the
  physical thrust and torque limits;
- what goes to the flight controller is **body rates + thrust** (thrust converted to throttle through
  the motor model), sent at 50 Hz by interpolating along the predicted trajectory;
- **Inertia compensation**: the flight controller's rate loop is tuned for the empty drone, so its
  response slows down with a payload. The NMPC amplifies "desired minus current body rate" by
  `(J + ΔJ) / J` before sending it (capped at 5×). This has the same effect as raising the rate-loop gain
  accordingly, without changing any flight-controller parameter.

### 7.4 What the controller does at release

- After sending the release command, the controller **does not clear the payload model right away**.
  It waits until the MHE's empty-payload confidence stays above 0.9 for 1 second, then confirms the
  release and switches back to the empty-drone model;
- if there is not enough evidence within 12 seconds, it enters an **unresolved state**: it keeps the
  current model, decelerates smoothly to a hover over 3 seconds, and collects evidence again at low
  dynamics;
- an accidental loss with no command is confirmed in the same way, as long as the estimator judges the
  drone empty with sufficient confidence.

## 8. How the quantities change during one mission

| Phase | What actually happens | MHE estimate | NMPC model and behavior |
| --- | --- | --- | --- |
| Empty flight | No payload | m ≈ empty mass, s ≈ 0, high empty-payload confidence | Empty-drone model |
| Descend, attach | Box attached but still supported by the ground | m essentially unchanged | Sends attach command; inertia first raised to the safe floor |
| Lift-off | Box leaves the ground and its weight is carried | m climbs to the loaded value, s appears, empty-payload confidence falls | Mass and offset follow the estimate; after carrying is confirmed, inertia is handed over to the MHE |
| Figure-8 | Sustained loaded flight | m and s stay near loaded values (s drifts slightly during the maneuver) | Tracks with the loaded model, outputting the trim torque automatically |
| Release | Box detaches | m drops back, s returns to zero, empty-payload confidence rises | Switches back to the empty-drone model once confidence stays above threshold |
| Jam / accidental loss | Command and reality disagree | Follows what physically happens | Jam: unresolved state and hover, model kept; accidental loss: confirmed from evidence on its own |

## 9. What it achieves (simulation results)

All results below are from PX4 software-in-the-loop simulation (Gazebo).

- **The mass estimate no longer gets stuck after lift-off.** With the old configuration, in about 7% of
  flights the mass estimate stayed on its lower bound after lift-off, the inertia fallback could never
  hand over, and the inner loop stayed over-gained and kept oscillating. The cause was aliasing from
  point-sampling the motor data. After switching to interval averaging and adding overrange
  re-anchoring, a 64-run pre-registered paired experiment showed the fraction of lift-phase frames on
  the bound dropping from **24.5% to 3.2%** (p < 0.001), with no increase in divergence.
- **Full-mission safety tests: 12/12 passed** (normal release, detachment delayed 4 s after the
  command, jammed gripper, accidental loss with no command; 3 runs each):
  - the payload model was never cleared while the box was still attached;
  - jammed gripper: 3/3 entered the unresolved state and hovered stably;
  - accidental loss: 3/3 detected on its own within about 3.5 seconds;
  - peak position error after the event was 0.14–0.24 m, with no divergence.
- **Estimation accuracy**: mass estimate about 2.20 kg while carrying (true 2.214 kg); the main offset
  component reaches about 86% of the true value.

## 10. Limitations

- These are simulation results only. Moving to hardware requires measured motor speeds from the ESCs
  and a re-calibrated thrust coefficient; with a thrust-coefficient error beyond about 5% the method
  clearly fails.
- Payload mass and offset can only be estimated as a product; the vertical distance r_z must be known.
- During aggressive maneuvers the first-moment estimate drifts slightly.
- Known edge case: if the box attaches only during the climb (forming a very long lever arm), the drone
  crashes, so the attach logic needs to forbid it; the release phase under the new configuration still
  needs a larger-sample paired validation.
