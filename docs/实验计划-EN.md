# Experiment Plan: Payload-Adaptive MHE–NMPC

## 1. Aim

I plan to test whether a drone can estimate changes in its payload and maintain stable flight during payload release.

The system uses Moving Horizon Estimation (MHE) to estimate payload properties and Nonlinear Model Predictive Control (NMPC) to update the control model. It checks flight measurements to confirm payload loss, including when no release command is received.

The experiments will answer three questions:

1. Can the system avoid assuming that the payload is gone before it actually detaches?
2. Can it detect payload loss without a release command?
3. Does estimating the payload’s first mass moment improve estimation stability?

The first mass moment is `s = payload mass × horizontal payload offset`. Unlike the offset alone, it naturally goes to zero when the payload is gone.

## 2. Main Experiments

| Test                           | What I will do                                                                                        | What I will check                                                   |
| ------------------------------ | ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| Estimation                     | Replay the same flight data using mass-only, mass + offset, and mass + first-moment estimators.       | Estimation accuracy, stability after release, and computation time. |
| Delayed or failed release      | Send a release command, then delay detachment by 0, 1, 4, or 8 seconds, or keep the payload attached. | Whether the controller switches to an unloaded model too early.     |
| Payload loss without a command | Detach the payload without notifying the controller.                                                  | Detection rate, detection delay, and flight stability.              |
| No loss or partial loss        | Keep the payload attached, or release only part of it.                                                | False alarms and incorrect declarations that the drone is unloaded. |
| Robustness                     | Add thrust-model errors, motor-speed measurement errors, and wind disturbances.                       | The conditions under which the system remains reliable.             |

I will start with hovering and straight-line flight. Turns and faster flight will follow after the basic tests work reliably.

## 3. Methods to Compare

| Method                                  | Release handling                                                              |
| --------------------------------------- | ----------------------------------------------------------------------------- |
| Command-only                            | Assumes release immediately after the command.                                |
| Command + evidence                      | Starts checking physical evidence after the command.                          |
| Proposed method                         | Continuously checks physical evidence, even without a command.                |
| Proposed method without moment evidence | Tests whether first-moment evidence helps prevent false release confirmation. |

The first three methods will use the same estimator and controller settings wherever possible, so the comparison focuses on release handling. A method using the true detachment time will provide an ideal reference in simulation. A standard change detector will also be compared at a similar false-alarm rate.

## 4. Measurements

For each test, I will record:

- Incorrect switches to the unloaded model before actual detachment.
- Successful detections, missed detections, and detection delay.
- False alarms while the payload remains attached.
- Position error, recovery time, and emergency-hover events.
- Estimator or controller failures and computation time.

Actual detachment time will be recorded independently. I will report failed trials as well as successful ones.

## 5. Execution Plan

1. Finish the comparison methods and check data logging.
2. Use a separate development dataset to tune the settings, then freeze them for testing.
3. Run log replay and repeated PX4 simulation tests under matching conditions.
4. Validate the key cases in hardware-in-the-loop or tethered tests.
5. Perform a small set of real-flight tests: normal release, delayed release, failed release, and loss without a command.

The initial real-flight target is a **0.15 kg payload**, a **5–10 cm horizontal offset**, and **hovering to 2 m/s flight**. Dangerous comparison cases will stay in simulation or restrained tests.

## 6. Expected Outcome

The goal is to show whether the proposed method avoids premature release confirmation, detects unexpected payload loss, and maintains stable flight. If release cannot be confirmed, the intended response is to stop the mission and hover safely. The results will also identify the method’s limitations.
