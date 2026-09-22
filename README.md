# GNC Learning

A personal, hands-on journey through **Guidance, Navigation & Control (GNC)** for unmanned air, surface, and underwater vehicles.

Everything here is built from first principles in Python (with a MATLAB reference implementation): rigid-body dynamics, quaternion attitude math, Kalman filters, complementary filters, PID control, and waypoint guidance, all tied together in closed-loop simulations.

The next phase follows a revamped study plan built around three companion books (see [Study plan](#study-plan-fall-2026--spring-2027)): state-space control and estimation, autonomy software (ROS 2 / PX4), and C/C++ for flight software.

---

## Progress at a glance

| Area | Status |
|---|---|
| Foundations: RK4, PID, linear Kalman filters, EKF (Tiers 0–1) | ✅ Done |
| Quaternion attitude dynamics, complementary filter, EKF AHRS (Tiers 1–2) | ✅ Done |
| Quadrotor 6-DOF GNC with guidance and sensors in the loop (Tiers 3–4) | ✅ Done |
| MATLAB quadrotor sim and its Python port (`MATLAB/`, `quad_sim_py/`) | ✅ Done |
| Lab 1: frames and rotations · Lab 2: 3-DOF USV · Lab 3: 6-DOF quadrotor | ✅ Done |
| State-space control and estimation labs (Lab 4–10), *GNC Notes* | ⏳ Next |
| Autonomy software: ROS 2, PX4 SITL, perception, *Autonomy Toolbook* | ⏳ Planned |
| C/C++ and Eigen for flight software, *The Machine Underneath* | ⏳ Planned |

---

## Completed work

### Repository layout

```
GNC-Learning/
├── governing_functions/   # Shared GNC library used by the Tier scripts
├── Tier0/                 # Foundations: RK4 integration, spring-mass, pendulum + PID
├── Tier1/                 # Estimation: linear Kalman filters, EKF, EKF-based AHRS
├── Tier2/                 # Attitude: quaternion rigid-body sim, complementary filter
├── Tier3/                 # Quadrotor 6-DOF GNC with quaternion attitude control
├── Tier4/                 # Quadrotor sim with noisy sensors in the loop (gyro/accel/GPS)
├── quad_sim_py/           # Self-contained Python port of the MATLAB quadrotor sim
├── MATLAB/                # Original MATLAB quadrotor sim (dynamics, PID, guidance)
├── CoursePlan/            # Original 6-month curriculum + Month 1 lab code and write-ups
│   └── Month1/            # Lab 1–3: frames, 3-DOF USV, 6-DOF quadrotor
└── GNC_Maritime_UAV_6Month_Curriculum.pdf
```

### Tiers: building the toolbox ✅

| Tier | Script | What it covers |
|---|---|---|
| 0 | `intro_learning.py` | Simulating spring-mass and pendulum systems with RK4 |
| 0 | `pendulum_PID_testing.py` | Driving a pendulum to a target angle with PID control |
| 1 | `kalman_filter_test.py` | 1-D and multi-D linear Kalman filters and an EKF on pendulum dynamics |
| 1 | `ekf_ahrs.py` | EKF-based attitude and heading reference system (AHRS) with gyro-bias estimation |
| 2 | `rigid_body_attitude_sim.py` | Quaternion rigid-body attitude dynamics with feedback control |
| 2 | `complementary_filter.py` | Gyro + accelerometer complementary filter compared with gyro-only integration |
| 3 | `quaternion_GNC_quadrotor.py` | Quadrotor 6-DOF sim with cascaded position/attitude control and multi-waypoint guidance |
| 4 | `quad_sim_sensors.py` | The Tier 3 quadrotor closed on *estimated* state from simulated noisy sensors |

### Quadrotor simulation stack ✅

- **`MATLAB/`**: `quadDynamics.m`, `quadControllerPID.m` (cascaded PID), and `guidanceMultiWaypoint.m` (cubic-spline waypoints with yaw), driven by `MainSimYaw.m`.
- **`quad_sim_py/`**: a line-by-line Python port of the MATLAB stack, extended with a sensor and estimation layer (`quad_sensors.py`).

### Month 1 labs (`CoursePlan/Month1/`) ✅

Each lab has a `.py` implementation and a `.md` write-up.

- **Lab 1: Reference frames and kinematics.** Euler angles, DCMs and quaternions, plus a comparison of aircraft and marine (Fossen) conventions.
- **Lab 2: 3-DOF USV simulation.** A Fossen-style surface vessel model with rigid-body plus added mass, Coriolis, linear and quadratic damping, and ocean current.
- **Lab 3: 6-DOF quadrotor simulation.** A 13-state model (NED position, quaternion, body velocities, body rates) driven by a desired trajectory.

### `governing_functions/`: the shared library

| Module | Contents |
|---|---|
| `integrators.py` | `rk4_step`: 4th-order Runge-Kutta integrator |
| `attitude_math.py` | Euler ↔ DCM ↔ quaternion conversions, `quat_multiply`, `quat_inverse`, `quat_error` |
| `dynamics.py` | Spring-mass, pendulum, 3-D kinematics, quaternion rigid-body attitude dynamics, rotor mixer, full quadrotor dynamics |
| `controllers.py` | `PIDController` |
| `estimators.py` | `KalmanFilter_1D`, `KalmanFilter_multiD`, `Extended_KalmanFilter` |
| `attitude_filters.py` | Sensor models (gyro, accelerometer, GPS), complementary filter, gyro-only propagation |
| `ahrs_models.py` | EKF process and measurement models for a quaternion + gyro-bias AHRS |
| `guidance.py` | `guidance_multi_waypoint`: cubic-spline trajectory through `[x, y, z, ψ]` waypoints |

---

## Study plan (Fall 2026 – Spring 2027)

Future work is organized around three companion books I'm working through. Each one pairs with the others: the math, the frameworks that run it, and the language those frameworks are written in.

### 1. *GNC Notes*: the math

A derivation-first companion to the labs, covering frames and quaternions, Newton–Euler dynamics, the Fossen marine model, quadrotor dynamics, state-space control, Kalman filtering, and orbit determination with GPS. Parts I–II back up the completed Labs 1–3. Parts III–IV drive the next set of labs:

| Lab | Topic | Builds on |
|---|---|---|
| 4a | Linearize `quadDynamics.m` about hover to get (A, B) | Lab 3, MATLAB sim |
| 4b | Controllability of the hover-linearized quad (Kalman rank test) | 4a |
| 5a | Observability of candidate no-GPS sensor sets | 4a |
| 6a | Pole-placement attitude controller (Ackermann / `place`) | 4a |
| 6b | PID vs. pole placement vs. LQR bake-off on the same model | 6a |
| 7a / 7b | Full-order and reduced-order Luenberger observers | 5a |
| 8 | EKF for GPS-denied localization | Tier 1 EKF, 5a |
| 8b | LQR + EKF → LQG, closing the loop on the estimate | 6b, 8 |
| 9 | GPS dropout and reacquisition stress test (EKF divergence) | 8 |
| 10 | Tie filter covariance to a stated navigation requirement | 9 |

Part IV also covers the two-body problem, Keplerian and equinoctial elements, GPS pseudorange/Doppler models, and batch vs. sequential least-squares orbit determination.

### 2. *Autonomy Toolbook*: the frameworks

The software stack that turns the math into a flying system, organized as layers of an autonomy stack:

- **Middleware:** ROS 2 (nodes, topics, services, actions), DDS and QoS, TF2, Nav2, `robot_localization`
- **Autopilots:** PX4 (uORB, flight modes), MAVLink and ground control stations, PX4–ROS 2 offboard control over uXRCE-DDS, ArduPilot, and SITL/HITL with Gazebo
- **Space flight software:** NASA cFS, JPL F Prime, and ground segments
- **Perception and GPS-denied localization:** the YOLO family, visual/multi-sensor localization, motion-capture ground truth
- **Edge AI compute:** neuromorphic hardware (BrainChip Akida) vs. GPU edge compute (Jetson)
- **Human–machine teaming:** natural-language mission commanding with LLMs, plus safety guardrails
- **Communications:** RF spectrum planning and multi-vehicle networking

First milestone: fly a basic mission in **PX4 SITL + Gazebo** through QGroundControl, then drive it from a ROS 2 node in Offboard mode.

### 3. *The Machine Underneath*: the language

C and C++ for aerospace software, starting from the Python/MATLAB background above:

- **Environment:** terminal, WSL, the compile/link toolchain, VS Code and git
- **How compiled programs run:** stack vs. heap, headers, the preprocessor, linking
- **C fundamentals:** static types, pointers, arrays and structs, `malloc`/`free` and common memory bugs
- **C++:** classes and RAII, references and const-correctness, templates and the STL, smart pointers, exceptions
- **Build and debug:** Make, CMake, `gdb`, sanitizers
- **Real-time and safety-critical C++:** determinism and avoiding the heap, MISRA, Eigen

Capstone: **port `quadDynamics.m` to C++ with Eigen** and cross-check it numerically against the MATLAB and Python versions in this repo.

> The original [6-month maritime/UAV curriculum](CoursePlan/GNC_Maritime_UAV_6Month_Curriculum.md) is kept for reference. The three books above replace it as the plan going forward.

---

## Getting started

### Requirements

- Python 3.9+
- `numpy`
- `matplotlib`

```bash
pip install numpy matplotlib
```

MATLAB is only needed for the files in `MATLAB/`.

### Running the simulations

Each Tier script adds the repo root to `sys.path`, so you can run it from anywhere:

```bash
python Tier0/pendulum_PID_testing.py
python Tier1/ekf_ahrs.py
python Tier2/complementary_filter.py
python Tier3/quaternion_GNC_quadrotor.py
python Tier4/quad_sim_sensors.py
```

The Month 1 labs are standalone:

```bash
python CoursePlan/Month1/Lab2.py
python CoursePlan/Month1/Lab3.py
```

The self-contained quadrotor port uses local imports, so run it from inside its folder:

```bash
cd quad_sim_py
python main_sim_yaw.py
```

For MATLAB, open the `MATLAB/` folder and run `MainSimYaw.m`.

---

## Conventions

- **Frames:** NED (North-East-Down) inertial frame. Body frame is x forward, y right, z down.
- **Integration:** fixed-step RK4 (`dt = 0.01 s` in most sims).
- **Quaternions:** `governing_functions` uses scalar-last `[qx, qy, qz, qw]`. The Month 1 labs (`Lab1.py`, `Lab3.py`) use scalar-first `[qw, qx, qy, qz]`, so check the ordering before mixing code between them.

---

## Key references

- Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*
- Stengel, *Optimal Control and Estimation*
- Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and Navigation*
- Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*
- Stroustrup, *A Tour of C++*, and Meyers, *Effective Modern C++*
- Official docs for ROS 2, PX4, NASA cFS, F Prime, and Eigen
