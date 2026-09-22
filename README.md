# GNC Learning

A personal, hands-on journey through **Guidance, Navigation & Control (GNC)** for unmanned vehicles — UAVs (multirotor and fixed-wing), unmanned surface vehicles (USVs), and unmanned underwater vehicles (UUVs).

Everything here is built from first principles in Python (with a MATLAB reference implementation): rigid-body dynamics, quaternion attitude math, Kalman filters, complementary filters, PID control, and waypoint guidance, all tied together in closed-loop simulations.

The learning path follows a 6-month, master's-level curriculum, [`GNC_Maritime_UAV_6Month_Curriculum`](CoursePlan/GNC_Maritime_UAV_6Month_Curriculum.md) (also available as a [PDF](GNC_Maritime_UAV_6Month_Curriculum.pdf)).

---

## Repository layout

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
├── CoursePlan/            # 6-month curriculum + monthly lab code and write-ups
│   └── Month1/            # Lab 1–3: frames, 3-DOF USV, 6-DOF quadrotor
└── GNC_Maritime_UAV_6Month_Curriculum.pdf
```

### `governing_functions/`: the shared library

Reusable building blocks, importable as a package:

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

### Progression by tier

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

### Course plan labs (`CoursePlan/Month1/`)

Each lab has a `.py` implementation and a `.md` write-up of what I learned.

- **Lab 1: Reference frames and kinematics.** Euler angles, DCMs and quaternions, plus a comparison of aircraft and marine (Fossen) conventions.
- **Lab 2: 3-DOF USV simulation.** A Fossen-style surface vessel model with rigid-body plus added mass, Coriolis, linear and quadratic damping, and ocean current.
- **Lab 3: 6-DOF quadrotor simulation.** A 13-state model (NED position, quaternion, body velocities, body rates) driven by a desired trajectory.

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

The course plan labs are standalone:

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
- **Quaternions:** `governing_functions` uses scalar-last `[qx, qy, qz, qw]`. The Month 1 labs (e.g. `Lab3.py`) use scalar-first `[qw, qx, qy, qz]`, so check the ordering before mixing code between them.

---

## Roadmap

Following the [curriculum](CoursePlan/GNC_Maritime_UAV_6Month_Curriculum.md):

| Month | Module | Status |
|---|---|---|
| 0 | Foundations refresher (Tiers 0–4) | ✅ |
| 1 | Vehicle dynamics: air, surface, underwater | 🚧 In progress |
| 2 | Navigation in GPS-denied / degraded environments | ⏳ |
| 3 | Guidance and motion planning (LOS, Dubins/RRT*, COLREGS) | ⏳ |
| 4 | Control for underactuated and disturbed systems | ⏳ |
| 5 | Integrated multi-domain systems (UAV + USV + UUV) | ⏳ |
| 6 | Capstone project | ⏳ |

Alongside the GNC work, the plan includes a programming track that ports these algorithms to **C/C++ (Eigen)** and **ROS 2**.

---

## Key references

- Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*
- Stengel, *Optimal Control and Estimation*
- Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and Navigation*
- Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*

The full reading list is in the curriculum.
