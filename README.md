# UR5 Arm: Computed-Torque vs PID Control in MATLAB/Simulink

Simulation of a 6-DOF Universal Robots UR5 performing a pick-and-place task, comparing a
**computed-torque (model-based) controller** against a **tuned independent-joint PID** baseline,
including a robustness study under model error and sensor noise.

![Pick-and-place animation](arm_pick_place.gif)

## Key result

With an unmodelled 2 kg payload, 10% error in link masses and encoder noise, computed-torque
control cut end-effector tracking error by **~70%** compared with PID, designed for the same
closed-loop bandwidth.

| Scenario | PID RMS [mm] | CTC RMS [mm] | RMS reduction | PID max [mm] | CTC max [mm] |
|---|---|---|---|---|---|
| Nominal (model = plant) | 4.96 | 2.1e-05 | 100.0%* | 26.7 | 3.5e-05 |
| 2 kg payload | 8.00 | 2.18 | 72.7% | 46.3 | 12.7 |
| Payload + 10% mass error | 8.53 | 2.58 | 69.7% | 49.5 | 15.1 |
| Payload + mass error + sensor noise | 8.53 | 2.59 | 69.7% | 49.5 | 15.1 |

\* With a perfect model, computed torque cancels the dynamics exactly, so error is only
numerical. This row is a sanity check, not a performance claim. The mismatch rows are
the meaningful comparison.

![RMS error by scenario](robustness_rms.png)
![Worst-case tracking error](robustness_worst_case.png)

## What the project covers

| Stage | Script | What it does |
|---|---|---|
| 1. Model | `step1_model_arm.m` | UR5 from DH parameters (no toolbox) and as a `rigidBodyTree`; FK sanity check, mass matrix check, workspace plot |
| 2. Plan | `step2_plan_trajectory.m` | 8 Cartesian pick-and-place waypoints, numerical IK with seed chaining and angle unwrapping, quintic joint trajectory at 1 kHz, speed and torque feasibility checks |
| 3. Control | `step3_compare_nominal.m` | Builds both Simulink models programmatically and compares PID vs computed torque on the nominal arm |
| 4. Robustness | `step4_robustness_study.m` | Re-runs both controllers with payload, mass error and sensor noise; writes the table above and figures |

Supporting functions: `build_ur5_model.m` (builds `ur5_pid.slx` / `ur5_ctc.slx`),
`simulate_ur5.m` (runs and scores one simulation), `ur5_params.m`, `dh_fk.m`,
`check_requirements.m` (verifies toolboxes).

## Method

**Plant.** Full rigid-body dynamics of the UR5 (including gravity) via the Robotics System Toolbox
*Forward Dynamics* block, joint torque saturation at the UR5 motor limits, and two integrators for
joint speed and angle.

**PID baseline.** Per-joint gains tuned from the diagonal of the mass matrix at the start pose:

$$\tau = K_p e + K_d \dot e + K_i \textstyle\int e, \quad K_p = M_{jj}\omega_n^2,\; K_d = 2\zeta\omega_n M_{jj},\; K_i = K_p\,\omega_n/10$$

**Computed torque.** Uses the arm's model at every step (the *Inverse Dynamics* block) to cancel
the nonlinear dynamics:

$$\tau = \hat M(q)\,\big(\ddot q_{ref} + K_p e + K_d \dot e + K_i \textstyle\int e\big) + \hat C(q,\dot q)\dot q + \hat G(q)$$

**Fair comparison.** Both controllers are designed for the same nominal closed-loop
polynomial $s^3 + 2\zeta\omega_n s^2 + \omega_n^2 s + \omega_n^3/10$ with $\omega_n = 20$ rad/s and $\zeta = 1$.
The controller always uses the *nominal* model; only the simulated plant is changed in the robustness study.

## Findings

- **PID error is dominated by gravity and coupling.** The shoulder and elbow (J2, J3) lag during fast
  moves even with a perfect model, because one fixed gain cannot match a configuration-dependent inertia.
- **Computed torque degrades gracefully.** Error rose from almost nothing to 2.6 mm as the model became
  wrong, while PID was already at 5 mm with a perfect model. Final settling error was about 11× lower (0.07 vs 0.84 mm).
- **Noise trade-off.** Encoder noise of about 0.006° barely changed tracking, but raised torque jitter
  about 30× for PID and 50× for computed torque. The model-based law scales feedback by $\hat M(q)$, which
  amplifies measurement noise on the heavy joints. On real hardware this calls for velocity filtering or an observer.

## Running it

**Requirements** (full list in [`requirements.txt`](requirements.txt)):

| Product | Needed for |
|---|---|
| MATLAB R2022a or newer | everything |
| Simulink | the closed-loop models |
| Robotics System Toolbox | robot model, IK, trajectories, dynamics blocks |
| Simscape Multibody *(optional)* | only the optional 3D model in step 1 |

```matlab
check_requirements   % verifies your installation
run_all              % runs step1 -> step4 in order, about 6-8 minutes
```

Or run `step1_...` to `step4_...` individually in order. Each one saves the `.mat` file the next step needs.

## Limitations and next steps

- Rigid joints with no friction, backlash or motor dynamics. Adding a joint-friction model would
  shrink the gap between the two controllers in a realistic way.
- The payload is unknown to the controller. **Adaptive computed torque** (online estimation of payload
  mass) or **robust sliding-mode control** would be the natural next comparison.
- Velocity is taken from the true state plus noise. A real implementation would differentiate encoder
  counts, so a filter or Kalman observer is needed.
- Constant-orientation pick-and-place only. Obstacle-aware planning (e.g. `manipulatorRRT`) is an obvious extension.
- Simulation only. The controller could be deployed via Simulink Coder or ROS 2 for hardware or Gazebo testing.

## Author

**Harish Venkatesan M R**, MSc Intelligent Robotics, University of York
[GitHub](https://github.com/harishvenky29) · [LinkedIn](https://www.linkedin.com/in/harish-venkatesan-m-r-833173191)
