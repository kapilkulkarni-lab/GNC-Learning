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
