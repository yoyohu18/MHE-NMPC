# Anticipated Questions from the Examiners · Prepared Answers

Companion to `MHE-NMPC-Presentation-EN.pptx`. Chinese version: [老师可能提问.md](老师可能提问.md).
Speaker notes: [逐页讲稿-EN.md](逐页讲稿-EN.md).

Ordered by how likely they are to be asked. **Questions marked ⚠ are dangerous** — either easy to
get wrong, or a weak answer makes it look as if you don't understand your own work.

---

## 1. Modelling and estimation theory

### Q1. Why MHE? Wouldn't a Kalman filter (EKF/UKF) do?

**Answer:** Three reasons.

1. **Constraints.** Mass has a physical lower bound (it cannot be lighter than the empty drone) and
   the first moment has a range limit. MHE is a constrained optimization, so these go in directly as
   `m ∈ [0.95 m_B, 5 kg]`. An EKF can only clip after the fact, and clipping breaks covariance
   consistency.
2. **Abrupt changes.** A payload step is not Gaussian process noise. An EKF "allows" a jump by
   inflating Q, which makes it noisy the rest of the time. MHE re-optimizes the whole trajectory in
   the window, so adapting to a step is built into its structure.
3. **Nonlinearity plus parameter augmentation.** With quaternion attitude and augmented parameters
   the model is strongly nonlinear, and the EKF's first-order linearization is visibly wrong under
   the large excitation of pickup and drop transients.

**The honest caveat to add:** the cost is computation. Our MHE has a 9.2 ms median and 59 ms 99th
percentile, about 7× the mass-only version.

---

### Q2. The MHE window is 2 seconds — doesn't that mean a built-in 2-second delay?

**Answer:** Two separate things.

- **The estimate itself does not wait 2 seconds.** It is solved every cycle (10 Hz) on the most
  recent 2 seconds of data, and outputs the estimate **at the current time**. After a step, new data
  enter the window frame by frame and the estimate climbs continuously; it does not wait for the
  window to fill.
- **Confirmation latency is a different matter.** The 3.5 s median during maneuvers reported in the
  paper comes mainly not from the window, but from **requiring the evidence to persist** (confidence
  > 0.90 for 1 s), and from the first-moment reference window possibly failing to form during the
  maneuver.

**⚠ Don't answer:** "a longer window is more accurate". Our own experiments refuted that — at 4 m/s,
N=10 gave a mass bias 3.7× smaller than N=20, and N=40 sat on the lower bound the whole time. The
mechanism: the longer the window, the longer the stretch of trajectory over which the "parameters
constant within the window" assumption must hold, and the worse that approximation becomes.

---

### Q3. Is mass really observable? When is it not?

**Answer:** Observability comes from **excitation**; it is not free.

- In hover, the thrust residual shows up mainly in the vertical channel, where mass and a sustained
  vertical disturbance (e.g. an updraft) are **fundamentally inseparable**. The paper lists this as a
  limitation.
- With an offset, things are actually much better. Our CRLB analysis shows that adding the first
  moment improves the lower bound on the mass estimate in offset hover from 0.0109 kg to
  0.0011 kg, **with 94% of the information coming through the rotational channel**. So estimating the
  first moment does not just keep the geometry well-defined; it materially improves the
  observability of mass itself.

---

### Q4. ⚠ Why estimate the first moment s instead of the CoM offset c directly?

**This is the central question of the whole work; the answer must be clean.**

**Answer:** Because `c` is undefined at the moment of release. Physically `s = m_P · r_xy → 0`, but
`r_xy` is 0/0 as `m_P → 0`.

And this is not just on paper: in the early mass-only version, the only way the optimizer could
suppress a spurious offset was to push total mass below the empty drone — **14 of 55 post-release
samples were pinned at the lower bound**.

With s instead: (a) the first moment is the standard linear parameterization of rigid-body inertial
parameters, which enter the dynamics linearly; (b) `c = s/m` can be computed without knowing the
grasp geometry; (c) it is the **minimal structure** we found that expresses payload presence, trim
torque, and roll/pitch inertia at the same time.

---

### Q5. Is the inertia ΔJ estimated too?

**Answer:** **No — say this clearly, or it will look as if you are overstating the online degrees of
freedom.**

The only online degrees of freedom are the mass m and the first moment s. ΔJ is generated
algebraically from m and a **weak prior on the vertical lever arm r_z** (the leading term of the
parallel-axis theorem). r_z is **structurally unobservable** from the thrust–torque channel, so it
has to be a prior.

The paper also states two approximations: the small O(r_xy²) diagonal terms and the yaw inertia
increment are dropped.

**If asked "what if the prior is wrong":** we tested it. A 20% error in the r_xy prior biases mass by
3.5%, while a 33% error in r_z changes mass by 0.00% — so the most sensitive quantity happens not
to be the least certain one in the prior.

---

### Q6. Quaternions have a unit-norm constraint — how does the optimizer handle it?

**Answer:** The quaternion is propagated as a state without an explicit normalization constraint;
it is held in place by the attitude terms in the cost.

**⚠ A pitfall we hit that is worth mentioning:** if the quaternion is allowed to be non-normalized,
`(|q|, m)` becomes **exactly degenerate** — the optimizer can "pretend the mass is lighter" by
"stretching q", the two are indistinguishable in the cost, and mass becomes unidentifiable. This was
a real problem, and it has been fixed.

---

## 2. Control and implementation

### Q7. Couldn't estimation errors make the controller less stable?

**Answer:** Yes, which is why the interface has five safeguards instead of writing the estimate
straight into the model:

1. health gating — when unhealthy, freeze the last valid model;
2. freshness — both the solution age and the topic receive age are checked;
3. low-pass filtering;
4. **separate rate limits on m / s / ΔJ before every solve**;
5. after rate-command scaling, a 5% margin is kept before saturation.

**But also admit:** we have observed one real failure chain. In one flight the estimate had been
unhealthy for 48 seconds before release, but the phase-timed release command went out anyway; after
release the model was mismatched and the flight diverged. **Release is not gated on estimator
health — a known gap, disclosed in the paper.**

---

### Q8. ⚠ Is reconstructing thrust from motor speeds reliable on real hardware?

**Answer:** **This is what we ourselves consider the top transfer risk, and we quantified it.**

We ran an ablation that only scales the estimator's thrust map by ±5%, and the interface **failed in
both directions**:
- ×0.95: the payload was **never recognized in 4 of 4** runs;
- ×1.05: a 0.09 kg phantom payload remained after release, and 3 of 4 runs stayed in UNRESOLVED.

So the conclusion is that **k must be calibrated to within 5%**; on hardware that means measuring the
RPM-to-thrust curve on a thrust stand.

Two simplifications in simulation should also be mentioned: the simulated plant uses **the same
square law**, and its rotor speeds are exact. Real hardware adds voltage sag, blade aerodynamics and
induced inflow (the paper cites Bangura & Mahony and states explicitly that we did not model the
last one).

---

### Q9. Why is the NMPC at 20 Hz and the MHE at 10 Hz? Isn't the rate mismatch a problem?

**Answer:** They don't need to run at the same rate. The MHE reads the latest state and inputs on
its own timer, and its output carries **its own timestamp and solution age**; the NMPC decides from
the age whether to trust it. The two solvers are decoupled, and each picks its rate from its own
compute budget.

The MHE runs at 10 Hz because it is expensive: 9.2 ms median, 59 ms 99th percentile, 72.8 ms peak.
**With a 100 ms period the tail margin is already narrow**; 20 Hz would overrun.

---

### Q10. What happens when a solver fails?

**Answer:** There are layered fallbacks. After several consecutive failures, the MHE
re-initializes and re-seeds the mass from the **vertical force balance** in the window (rotor thrust
against odometry acceleration).

One detail worth mentioning: after re-anchoring, the first moment is initialized to zero, and **that
zero must not be taken as "collapse evidence"** — otherwise a solver failure would masquerade as a
release. The code explicitly excludes this case.

---

### Q11. Why not just change PX4's rate-loop gains directly?

**Answer:** We tried; it was worse.

Rewriting flight-controller parameters at high rate has two problems: (a) PX4 **saves** them as real
settings, which contaminates the next clean start; (b) the gain jump is itself a disturbance.

The current approach is **continuous scaling of the body-rate command**, with target hysteresis and
asymmetric rise/recovery rate limits, and **no PX4 parameters are rewritten**.

---

## 3. Experimental methodology

### Q12. What exactly does 32/36 prove?

**Answer:** **Narrow it yourself — that is better than waiting for an examiner to narrow it.**

It is a conditional closed-loop result for **the whole interface** (conditioned on the payload
physically attaching and the drone surviving to the release command), **not a validation of any
single release detector**, because the 32 confirmations took three different paths: 19
moment-gated, 4 mass-domain, 9 on confidence alone.

That is exactly how the paper phrases it. The 95% Clopper–Pearson interval is [73.9%, 96.9%]; the
interval is still wide, and 36 flights cannot support a stronger claim.

---

### Q13. Only 10/18 uncommanded losses were detected — isn't that too low?

**Answer:** It is low, and I won't dodge that. But the comparison arm is **0/18, and that zero is by
construction** — a command-armed confirmer is never activated without a command. The difference is
"can versus cannot", not "good versus better".

Seven of the eight misses share one cause: **when the loss happened, the loaded first-moment
reference window had not formed yet, or the payload had never been recognized.** So the direction
for improvement is clear — make the reference window form more reliably, not loosen the criterion.

---

### Q14. ⚠ Were the thresholds tuned on the same data? (the overfitting challenge)

**Answer:** No. The paper uses a full table (Table I) to document where every threshold came from.

- All thresholds were fixed in code **before** the final batch started and were not re-tuned on it;
- only two values were chosen against data — the first-moment collapse ratio and the residual
  floor — and both used **early flights disjoint from the final batch**;
- the collapse ratio of 0.30 came from **leave-one-out cross-validation** on 59 early flights, and
  every fold picked 0.30;
- the rest are engineering choices, and the paper states plainly that "we did not sweep their
  closed-loop sensitivity".

---

### Q15. Only n=5 per cell — is that statistically enough?

**Answer:** It depends:

- **For safety metrics we used a large sample**: 209 loaded flights, 3.7 hours, zero false
  releases, with a one-sided 95% upper bound of 1.4% per flight.
- **The comparison metrics really are small-sample**, which is why the paper **deliberately claims
  no transient advantage** in the L1 table — two cells have p = 0.008 and 0.012, but they don't
  survive correction for eight comparisons, so we don't claim them.
- The paired design was pre-registered, not an arm picked after the fact.

---

### Q16. It's SITL only — how much can we trust the conclusions?

**Answer:** I won't claim anything beyond simulation. The first limitation under the paper's title
is "All system-level evidence is presently SITL-only".

What can be claimed: **in the same simulation environment, command-triggered and evidence-triggered
semantics reach opposite conclusions when command and physics disagree.** That comparison is valid
in itself, because both arms run on the same platform.

What cannot be claimed: any absolute reliability number.

We also list the simulation-fidelity issues ourselves: gz-sim cannot change rigid-body inertia at
runtime (issue #2733), so every payload change has to come from the DetachableJoint platform; the
payload is rigidly attached, and gripper compliance and swing modes are not modelled.

---

### Q17. Why not compare against a fixed-parameter NMPC?

**Answer:** **That is a missing baseline the paper itself lists, and I won't argue it away.** The
table names three gaps explicitly: a fixed-parameter NMPC, a CUSUM detector with a matched
false-alarm rate on the same signals, and the combination "final estimator + L1".

---

## 4. Related work (⚠ prepare in advance; examiners who know the area will ask)

### Q18. ⚠ How is this different from NeuroMHE (T-RO 2024)?

**Answer:** NeuroMHE learns to **output the MHE weighting matrices online**, addressing the
bias–variance trade-off, but what it estimates is a **lumped disturbance force**. We estimate **named
physical parameters** (mass, first moment, inertia).

The practical meaning of the difference: a lumped disturbance force **cannot** be used to update the
thrust-margin constraints or the inner-loop gain scheduling, while named parameters can. The paper
says this in its Related Work.

**Risk note:** if we later add something like a "forgetting factor", it would overlap with
NeuroMHE's idea and we would need to reposition.

---

### Q19. ⚠ People have already used MHE to estimate mass and inertia for NMPC — what is new here?

**Answer:** First, acknowledge it: online estimation of inertial parameters for MPC has prior work
(Svacha 2020, Wüest 2019, Böhm 2021, and Mellinger 2011 for aerial manipulation).

Our differences are **three, and none of them is "a more accurate estimator"**:
1. **First-moment parameterization**, which keeps the geometry well-defined when the payload
   disappears — prior work estimates the CoM, which is ill-posed at the moment of release;
2. **Release semantics**: the controller clears its model only on sustained evidence, and enters
   UNRESOLVED and brakes when evidence is missing. That is a question of "when can the estimate be
   trusted", not "how accurate is it";
3. **Full-lifecycle, complete reporting**: grasp, carry, release — every attempt is reported,
   together with the confirmation path each one took, including failures at the actuator limits.

**If an examiner names a specific paper we didn't cite:** say honestly "I need to go back, read it,
and position it in the related work" — don't improvise.

---

## 5. On AI assistance (⚠ this will be asked — prepare it)

### Q20. ⚠ AI wrote this code — how much of it do you actually understand?

**Suggested answer — concrete, verifiable, not vague:**

> AI assistance was used for code generation, refactoring, adding tests and organizing
> documentation; I am responsible for the experimental design, the choice of criteria and the
> conclusions.
>
> My current focus is on systematically understanding and verifying the generated results. Evidence
> I can show:
>
> - I can explain where each threshold came from and which experiment fixed it (that is how Table I
>   was put together);
> - I can explain why the fast path was removed — it caused 3 early releases in 9 flights, and its
>   design assumption, "the mass domain never reads empty while loaded", is wrong at 4 m/s;
> - while going through the system I found an inconsistency between documentation and code: the
>   Chinese README still says "the MHE is diagnostic only and does not feed back into the
>   controller", while in the code the estimates have long been in the NMPC loop. **I found this by
>   reading the code; reading the documentation would have misled me.**
>
> In this talk I distinguish three kinds of content: what has been confirmed by code and tests, what
> holds only in simulation, and what still needs checking.

**⚠ Don't say:** "AI wrote all of this, so I'm not sure" — or, the other way round, pretend you wrote
every line yourself. The first gives up the defence; the second collapses as soon as someone asks
for details.

---

### Q21. So could you modify this system yourself?

**Answer:** Yes — and give a concrete example (pick one you have really changed):

- the release criterion is a pure function shared by the online node and offline replay, so a
  criterion change can first be validated by replaying frozen logs before flying it;
- all parameters live in `*_params.py`, and the comments state which experiment fixed each default;
- there are 30 standalone tests that don't need the simulator, most of which run in seconds.

---

### Q22. What do you do when results don't match your expectations?

**Answer:** Real examples are the strongest, for instance:

- we originally believed "a longer MHE window is more robust to noise", and a factorial experiment
  under aggressive maneuvering refuted it (N=10 was 3.7× better than N=20). **We did not change the
  default right away but marked the conclusion as "not settled"**, because in that batch window
  fraction and trajectory arc length were collinear, and the sample was too small;
- in the L1 comparison two cells had significant p-values but did not survive multiple-comparison
  correction, and **we chose not to claim them**.

---

## 6. Quick fact card (quote directly when asked for a number)

| Question | Number |
| --- | --- |
| NMPC | 13 states / 4 inputs, N=20 × 0.05 s, 20 Hz, 1.3 ms median (2.6% of the period) |
| MHE | 16 states, N=20 × 0.1 s, 10 Hz, 9.2 ms median / 59.2 ms p99 / 72.8 ms peak |
| Release completed | 32/36, 95% CI [73.9%, 96.9%] |
| Confirmation latency | 3.46 s median during the maneuver; 16.56 s median after braking (47% took the latter) |
| Confirmation paths | moment-gated 19 / mass-domain 4 / confidence only 9 |
| Delayed detach | arm A 5/5 early (median 4.01 s early); arm B 0/6 |
| Uncommanded loss | arm B 10/18; arm C 0/18 |
| False release | 0 / 209 flights (3.7 h), one-sided 95% upper bound 1.4% |
| Partial loss | 2 m/s tracked 14/15 levels; 4 m/s only 12/15, **pre-registered criterion not met** |
| L1 comparison | 60/60 stable; recovery 8–32% slower (median 16%); **no** transient advantage claimed |
| Thrust-coefficient sensitivity | fails at ±5% (0.95 → 4/4 unrecognized; 1.05 → 0.09 kg phantom) |
| Stress case | 0.3 kg / 4 m/s: hover uses 74% thrust, but pitch torque saturated in 98% of windows |
