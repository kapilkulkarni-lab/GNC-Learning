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
