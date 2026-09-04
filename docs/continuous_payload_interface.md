# Continuous MHE → NMPC payload interface

The default adaptive path has no payload-event input:

```text
motor speed + odometry
          ↓
  MHE: m, s=m_P r_xy, J
          ↓ /mhe/payload_estimate
  NMPC model + rate-loop scheduling
```

`/mhe/payload_estimate` is a `std_msgs/Float64MultiArray`. Its atomic field
order is defined by `offboard_test_acados.payload_estimate.FIELDS`:

```text
[m_total, s_x, s_y, c_x, c_y,
 dJ_xx, dJ_yy, dJ_zz, J_xx, J_yy, J_zz,
 no_payload_confidence, healthy, solution_age_sec]
```

`healthy` requires a successful latest solve and fresh odometry, motor-speed,
and known-input streams. `solution_age_sec` measures estimator-solution age,
not topic heartbeat age. NMPC additionally checks local topic receive age;
unhealthy or stale frames hold the last model and break the no-payload
confidence timer.

`c_xy=s_xy/m_total`. `dJ/J` is derived with the rack's vertical-arm prior;
the MHE uses `moment_a_mode=frozen` so mass cannot be used as an instantaneous
inertia knob inside a solve. The published states are low-pass filtered and
are never cleared by attach/drop commands.

NMPC slew-limits `m`, `s`, and `dJ` before each solve. Rate-loop adaptation
uses `dJ` through continuous body-rate-error scaling with target hysteresis,
low-pass filtering, and asymmetric rise/recovery slew limits. It does not
rewrite PX4 rate-gain parameters in the default path. Before publishing a
scaled roll/pitch command, a common scale is reduced to preserve 5% body-rate
headroom; the final hard command limit remains a last-resort guard.

There is one attach-time safety bootstrap for inertia only. When NMPC issues
the gripper-enable command, it ramps the roll/pitch `dJ` floor to the rack
envelope (`0.3 kg` by default). With `m_B=2.0643 kg` and `d=0.47 m`, the same
reduced-mass algebra as the plant model gives `dJ=0.0579 kg m²` and
`Jxx=Jyy≈0.0721 kg m²`; rate scaling is capped at `5x`. This does not inject
payload mass or `s_xy`. After persistent coherent MHE payload evidence, the
floor fades out over `0.5 s` and MHE `dJ` becomes authoritative. This bootstrap
uses NMPC's own attach command, not an attach notification; the drop path
continues to consume estimates only.

A planned drop enters a pending state after the physical release command. L1
state is reset and the drop state is completed only after
`no_payload_confidence` remains above its threshold for the configured hold
time. Confidence is the product of independent mass, first-moment, and inertia
empty scores, so a non-zero `s_xy` vetoes a false empty decision even if the
mass estimate touches its lower bound.

Legacy event-driven subscriptions remain available only when
`continuous_payload_estimates:=false` (NMPC) or
`external_event_inputs:=true` (MHE).
