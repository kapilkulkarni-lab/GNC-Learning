# GNC for Unmanned Maritime & UAV Systems
## A 6-Month, Master's-Level Curriculum (USV / UUV / UAV Specialization)

**Goal:** Build the GNC skillset needed for autonomy/GNC engineering roles at companies and labs working on unmanned surface vehicles (USVs), unmanned underwater vehicles (UUVs/AUVs), UAVs (multirotor and fixed-wing), and — increasingly in demand — **heterogeneous multi-domain teams** (e.g., a UAV scouting, a USV relaying, a UUV inspecting). This targets defense primes (e.g., ocean robotics, maritime autonomy), commercial ocean survey companies, UAV/drone autonomy companies, and research labs.

**Time commitment:** ~20–25 hrs/week, 26 weeks.

**Prerequisites:** Same as the base program — linear algebra, ODEs, classical mechanics, intro controls, programming (Python strongly preferred for this track — the open-source robotics stack is Python/C++ heavy).

---

## Tools You'll Need (This Track)
- **Python** (NumPy/SciPy, `control`, `filterpy`, `casadi`) — primary language for this ecosystem
- **ROS 2** — the lingua franca of robotics integration; you don't need to be a ROS expert but you must be conversant
- **Gazebo / Gazebo Sim** — physics simulation for UAV and marine vehicles
- **ArduPilot SITL** (Software-In-The-Loop) and/or **PX4 SITL** — the two dominant open-source autopilot stacks; both support multirotor, fixed-wing, and **Rover/Sub** vehicle types, so you can prototype UAV *and* USV/UUV control in the same ecosystem
- **MOOS-IvP** — MIT's open-source **marine autonomy** middleware, widely used in academic and some commercial USV/UUV work; worth knowing even if your employer uses something proprietary
- **UUV Simulator / Stonefish** — underwater vehicle dynamics simulators (hydrodynamics, sonar, currents) built on Gazebo
- **QGroundControl** — ground control station software for PX4/ArduPilot, useful for visualizing missions
- MATLAB/Simulink optional but common in defense-sector jobs — if you have access, replicate a few labs there too

---

## Programming Skills Track: Strengthening Python & Learning C/C++

**Why both languages:** In this field, Python dominates prototyping, estimation/guidance algorithm development, and Monte Carlo/data analysis — it's fast to write and easy to debug. But the vehicles themselves run on **C/C++**: ArduPilot and PX4 flight stacks, ROS 2 middleware, and most production embedded flight software are written in C++ (with C still common at the lowest embedded/driver level). A GNC engineer who's only fluent in Python is boxed out of a large share of jobs — especially anything touching flight software, embedded autopilot code, or ROS 2 package development. The plan below layers C, then C++, in alongside your existing GNC labs so you're never learning syntax in a vacuum — every new language skill is immediately applied to a GNC problem you've already built.

**How it works:** Each month has a "Programming Milestone" that piggybacks on that month's GNC lab — you're not doing separate toy exercises, you're re-implementing or extending your own GNC code in a new language, then comparing outputs to your Python version (a great habit: cross-validating two independent implementations catches bugs neither one would catch alone, and it's exactly how real teams validate flight software against a sim).

### Language Resources
- **C:** Kernighan & Ritchie — *"The C Programming Language"* (2nd ed.) — short, dense, still the best starting point
- **C++:** Stroustrup — *"A Tour of C++"* (fast, modern overview) then Meyers — *"Effective Modern C++"* (idiomatic, industry-standard practices)
- **Eigen** — the C++ linear algebra library used throughout robotics/GNC C++ code (matrix/vector ops, exactly what you need for filters and control laws); its docs double as a good tutorial
- **Python leveling-up:** Ramalho — *"Fluent Python"* — since you already know the basics, this is the book that takes you from "writes working code" to "writes idiomatic, efficient Python" (generators, decorators, proper OOP, vectorization patterns)
- **Tooling to pick up alongside the languages:** `gcc`/`g++`, `CMake` (build system used by ROS 2, PX4, ArduPilot, and basically everything else in this ecosystem), `gdb` (debugger), `valgrind` (memory-error checking — non-negotiable once you're writing C/C++ with manual memory management), `GoogleTest` (C++ unit testing), `clang-format` (style consistency)

### Month-by-Month Programming Ladder

| Month | Python Focus | C/C++ Focus | Applied To |
|---|---|---|---|
| 0 | Idiomatic NumPy, vectorized ops, classes/OOP refresh | C fundamentals: syntax, compilation (`gcc`), pointers, arrays vs. pointers, structs, manual memory (`malloc`/`free`) | Rewrite a small existing Python script (e.g., the quaternion/DCM conversion utility) in plain C to internalize memory management basics |
| 1 | — | C++ fundamentals: classes, references vs. pointers, RAII, STL containers (`vector`, `map`), compiling with CMake | **Port your 6-DOF vehicle plant (USV or quadrotor) from Python to C++.** Same integration loop, same physics — compare trajectories numerically to confirm they match |
| 2 | — | Templates, operator overloading (handy for vector/matrix types), intro to **Eigen** | **Re-implement your EKF in C++ using Eigen.** Feed it the same synthetic measurements as your Python filter and confirm the estimates match within numerical tolerance — this is a real validation habit, not just a coding exercise |
| 3 | — | ROS 2 basics in C++: nodes, publishers/subscribers, message types, building with `colcon` | **Wrap your LOS guidance law as a ROS 2 C++ node** — subscribe to a simulated pose topic, publish a heading command topic. This is your first real taste of how guidance code is actually deployed |
| 4 | Profiling Python (where it's slow and why) | Real-time/performance concepts: fixed-step timing loops, profiling (`perf`/`gprof`), why flight code avoids dynamic allocation and unbounded loops; a conceptual look at embedded C constraints (interrupts, fixed-point arithmetic) since this is how actual autopilot firmware is written | **Profile and tighten your C++ controller/EKF loop to hit a fixed real-time budget** (e.g., must reliably complete in under 5 ms) — a genuinely common flight-software interview topic |
| 5 | Python-side Monte Carlo harness, ROS 2 Python nodes for quick scripting/analysis | Read (not necessarily modify) a slice of real production code: PX4 or ArduPilot's estimator/controller source, or MOOS-IvP's C++ architecture — seeing real, deployed GNC C++ is worth more than another toy exercise at this point | **Build a small ROS 2 package mixing a Python simulation/analysis node with your C++ guidance+control node** — real Python/C++ interop inside one system, which is exactly how many actual robotics stacks are organized |
| 6 (Capstone) | — | — | **Decide your capstone's language split deliberately:** pure Python is faster to finish and fine if you're targeting research-leaning roles; pure C++ signals strength for embedded/flight-software-leaning roles; a **hybrid** (C++ for the real-time plant/nav/control core, Python for the Monte Carlo wrapper and analysis/plotting) most closely mirrors how real programs are actually built, and is the recommended default unless you have a specific reason to choose otherwise |

**A note on pacing:** don't try to "finish learning C++" before moving on — you won't, and you don't need to. Real proficiency here comes from repeatedly re-implementing GNC code you already understand conceptually, in a new syntax, under a deadline. By month 6 you'll have touched pointers, RAII, templates, Eigen, and ROS 2 C++ nodes — that's a genuinely credible amount of C++ for a first pass, and enough to keep growing on the job.

---

## Core Textbooks (This Track)
1. **Fossen, T. — "Handbook of Marine Craft Hydrodynamics and Motion Control"** (THE reference for marine vehicle GNC — surface and underwater)
2. **Antonelli, G. — "Underwater Robots"** (UUV/AUV modeling, navigation, control)
3. **Beard, R. & McLain, T. — "Small Unmanned Aircraft: Theory and Practice"** (the standard UAV GNC textbook, freely available companion site with MATLAB code)
4. **Stengel, R. — "Optimal Control and Estimation"** (still your estimation/optimal-control backbone)
5. **Bar-Shalom, Li, Kirubarajan — "Estimation with Applications to Tracking and Navigation"** (Kalman filtering reference)
6. **Farrell, J. — "Aided Navigation: GPS with High Rate Sensors"** (INS/GPS integration; also foundational for DVL/USBL-aided nav)
7. **LaValle, S. — "Planning Algorithms"** (free online — RRT*, sampling-based motion planning, essential for both UAV and marine obstacle avoidance)
8. **Groves, P. — "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"** (deep dive on GPS-denied and multi-sensor fusion)
9. Reference papers: MOOS-IvP publications (Newman, Benjamin et al.), DARPA ACTUV/Sea Hunter program papers, REMUS/Bluefin AUV nav papers (unclassified), Saildrone technical publications

---

## Program Structure

| Month | Module | Focus |
|---|---|---|
| 0 (buffer) | Foundations Refresher | Same as base program |
| 1 | Vehicle Dynamics — Air, Surface, Underwater | Marine craft (Fossen model), quadrotor, fixed-wing UAV dynamics |
| 2 | Navigation in Denied/Degraded Environments | DVL/USBL/LBL underwater nav, VIO/SLAM for GPS-denied UAV, comms-constrained fusion |
| 3 | Guidance & Motion Planning | LOS path following, Dubins/RRT*, COLREGS-aware collision avoidance, multi-vehicle task allocation |
| 4 | Control for Underactuated & Disturbed Systems | Backstepping/sliding-mode marine control, quadrotor/fixed-wing autopilots, wave/wind rejection |
| 5 | Integrated Multi-Domain Systems | Heterogeneous UAV+USV+UUV mission simulation, Monte Carlo, comms/latency modeling |
| 6 | Capstone Project | Full heterogeneous mission GNC design + portfolio + interview prep |

---

## MONTH 0 (Optional) — Foundations Refresher
Same as the general program: linear algebra, ODEs, rotations (DCM/Euler/quaternion), classical control refresh.
**Added for this track:** Skim ROS 2 concepts (nodes, topics, services, tf2 transforms) and get ArduPilot SITL + PX4 SITL running locally — half a day of setup now saves you frustration in Month 5.

---

## MONTH 1 — Vehicle Dynamics: Air, Surface, and Underwater

### Week 1: Reference Frames & Kinematics (shared foundation)
- NED, body frame, ECEF; DCMs and quaternions
- **New here:** the marine convention (Fossen's SNAME notation: surge/sway/heave, roll/pitch/yaw, {n}-frame vs {b}-frame) — note where it overlaps and differs from aerospace convention
- **Lab:** Implement frame transformations for both an aerospace body-frame convention and Fossen's marine convention; write a short comparison note (this trips people up constantly when switching domains)

### Week 2: Marine Vehicle Dynamics (Fossen Model)
- 6-DOF marine vehicle equations of motion: rigid-body dynamics + hydrodynamic added mass + damping (linear/quadratic drag) + restoring forces (buoyancy/gravity for underwater vehicles)
- Underactuation: most USVs/UUVs only actuate surge force + yaw moment (no direct sway control) — this is the single biggest conceptual difference from aircraft/spacecraft GNC
- Environmental disturbances: ocean currents (modeled as body-frame velocity offset), wave forces (first-order/second-order, relevant mainly for USVs)
- **Lab (Milestone Project #1a):** Implement a 3-DOF (surge-sway-yaw) underactuated USV simulation using Fossen's model with current disturbance; validate against known step-response behavior

### Week 3: UAV Dynamics I — Multirotor
- Quadrotor/multirotor equations of motion: thrust/torque mixing, rotor dynamics, the classic "differentially flat" structure of multirotor dynamics
- Wind disturbance modeling, ground effect (brief)
- **Lab:** 6-DOF quadrotor nonlinear simulation with a simple rotor speed-to-thrust/torque mixer; verify hover trim and step responses

### Week 4: UAV Dynamics II — Fixed-Wing & UUV Depth/Buoyancy
- Fixed-wing 6-DOF equations (same aerospace formulation as manned aircraft), stability derivatives, trim conditions, coordinated turns
- UUV-specific: buoyancy control, depth/pitch coupling, ballast/trim systems, glider dynamics (for underwater gliders like Slocum) as a special low-power case
- **Lab (Milestone Project #1b):** Build a fixed-wing UAV 6-DOF sim AND a UUV depth-control plant (5-DOF: surge, heave, pitch, yaw + depth); this gives you three working vehicle plants (USV, UAV, UUV) to carry forward all program

---

## MONTH 2 — Navigation in GPS-Denied / Degraded Environments

### Week 5: Estimation Theory Refresher
- Bayesian estimation, batch and recursive least squares
- **Lab:** Recursive least-squares position estimate from simulated noisy range fixes (applicable to both acoustic ranging and UAV beacon-based nav)

### Week 6: The Kalman Filter Family
- Linear KF, EKF, UKF — derivations and when each is appropriate
- Observability issues specific to underactuated/degraded sensing (e.g., unobservable heading with only 2D GPS on a symmetric hull)
- **Lab:** EKF for a UAV fusing IMU + occasional GPS fixes; demonstrate covariance growth during GPS dropout and re-convergence on GPS reacquisition

### Week 7: Underwater Navigation (No-GPS-Ever Problem)
- Dead reckoning with a **Doppler Velocity Log (DVL)** — bottom-lock velocity measurement, error growth characteristics
- Acoustic positioning: **USBL** (Ultra-Short Baseline, ship-relative fix) and **LBL** (Long Baseline, seafloor transponder network) — geometry, accuracy trade-offs, update-rate limitations
- INS/DVL/USBL fusion via EKF; terrain-relative navigation (bathymetric map matching) as a GPS-free alternative
- **Lab (Milestone Project #2a):** Build an INS/DVL EKF for a simulated UUV transect; show unbounded position drift with DVL-only dead reckoning, then show bounded error when periodic USBL fixes are added

### Week 8: GPS-Denied Navigation for UAVs — VIO & SLAM
- Visual-Inertial Odometry (VIO): camera + IMU fusion, why it drifts (scale/bias observability issues), when it's "good enough" vs. when you need absolute references
- SLAM concept (simultaneous localization and mapping) — enough depth to discuss EKF-SLAM vs. graph-based SLAM (factor graphs) conceptually, without needing a full implementation
- Comms-constrained/multi-vehicle estimation: acoustic modems (underwater, ~kbps, seconds of latency) vs. RF (air, near-instant) — how this shapes what you can and can't fuse in real time across a heterogeneous team
- **Lab:** Simulate a UAV VIO+IMU EKF with realistic drift; compare position error growth against the UUV DVL case from Week 7 — write a short comparative memo (a favorite interview topic: "how is underwater nav different from GPS-denied air nav?")

---

## MONTH 3 — Guidance & Motion Planning

### Week 9: Line-of-Sight (LOS) Path Following — The Marine/UAV Workhorse
- LOS guidance law (Fossen formulation): cross-track error, lookahead distance, heading command generation — this single algorithm is used constantly in real USV/UUV/fixed-wing UAV autopilots
- Waypoint switching logic, path parameterization
- **Lab:** Implement LOS guidance driving your Month-1 USV plant along a waypoint path with a cross-current disturbance; tune lookahead distance and analyze cross-track error

### Week 10: Sampling-Based Motion Planning
- Dubins paths (minimum-turn-radius paths for fixed-wing/USV kinematic constraints)
- RRT and RRT* — sampling-based planning for obstacle-rich 2D/3D environments
- Potential-field and velocity-obstacle methods (fast, reactive, good for dynamic obstacles)
- **Lab:** Implement RRT* path planning for a UAV in a cluttered 3D environment (static obstacles) and re-plan when a new obstacle appears mid-mission

### Week 11: Collision Avoidance & Regulatory Constraints
- **COLREGS** (International Regulations for Preventing Collisions at Sea) — how autonomous USVs must encode give-way/stand-on rules; this is a genuinely unique domain requirement not found in aerospace GNC
- Velocity obstacle methods adapted for COLREGS-compliant avoidance
- UAV airspace constraints: geofencing, FAA Part 107/BVLOS considerations (know the vocabulary even if you're not doing the legal work)
- **Lab (Milestone Project #3a):** Extend your Week 9 LOS-guided USV sim to include a COLREGS-compliant avoidance maneuver against a crossing "give-way" contact

### Week 12: Multi-Vehicle Coordination & Task Allocation
- Formation control basics (leader-follower, virtual structure)
- Task allocation for heterogeneous teams (auction-based/market-based allocation is the common industry approach — conceptually simple, worth knowing cold)
- Communication-aware planning: replanning under intermittent/low-bandwidth links (critical for UUV-involving teams)
- **Lab:** Simple market-based task allocator assigning 3 simulated vehicles (1 UAV, 1 USV, 1 UUV) to 5 mission waypoints, minimizing total mission time subject to each vehicle's kinematic constraints

---

## MONTH 4 — Control for Underactuated & Disturbed Systems

### Week 13: Modern Control Refresher + Why Marine/UAV Vehicles Are Different
- State-space, controllability/observability, LQR refresher
- The underactuation problem revisited: you cannot just "invert" a marine vehicle's dynamics the way you can a fully-actuated spacecraft — nonlinear techniques matter more here
- **Lab:** LQR heading autopilot for your USV plant (surge/yaw sub-system); note the limitations when current disturbance is large

### Week 14: Backstepping & Sliding Mode Control for Marine/Underwater Vehicles
- Backstepping control design — the dominant nonlinear technique in marine GNC literature (handles underactuation and nonlinear damping terms directly)
- Sliding mode control — robustness to unmodeled hydrodynamics and current disturbances, chattering mitigation (boundary layer method)
- **Lab (Milestone Project #4a):** Design a backstepping heading + speed controller for your USV plant; compare disturbance rejection against the Week 13 LQR baseline under current disturbance

### Week 15: UAV Autopilot Design — Multirotor & Fixed-Wing
- Cascaded control architecture (the real-world pattern): inner attitude-rate loop → attitude loop → velocity loop → position loop, as used in PX4/ArduPilot
- PID + feedforward for multirotor position control; total energy control system (TECS) concept for fixed-wing altitude/airspeed control
- Wind disturbance rejection, actuator saturation handling (integrator anti-windup — a very commonly asked interview topic)
- **Lab:** Implement a cascaded position controller for your quadrotor plant; fly a simulated square-waypoint mission in wind; separately, implement a basic fixed-wing altitude-hold/airspeed-hold autopilot

### Week 16: Depth/Pitch Control for UUVs & Wave-Disturbance Rejection for USVs
- UUV depth-keeping and diving control (coupled depth/pitch dynamics, ballast dynamics)
- USV wave-disturbance rejection: notch filtering wave-frequency motion out of the control loop (a classic technique — avoids "fighting the waves" and wasting actuator effort/fuel)
- **Lab:** Depth-hold controller for your UUV plant with a simulated current-induced pitch disturbance; wave-filtering position-keeping controller for your USV plant in simulated sea state

---

## MONTH 5 — Integrated Multi-Domain Systems

### Week 17: Closing the Loop Per-Vehicle
- Integrate each vehicle's own navigation filter + guidance law + controller (USV: DVL/GPS EKF + LOS/COLREGS guidance + backstepping control; UAV: VIO/GPS EKF + RRT*/LOS guidance + cascaded autopilot; UUV: DVL/USBL EKF + LOS guidance + depth/heading backstepping control)
- **Lab (Milestone Project #5a):** Three independent closed-loop sims, one per vehicle type, each completing an assigned waypoint mission autonomously

### Week 18: Heterogeneous Team Integration
- Shared mission architecture: how a UAV (fast, wide-area, RF comms), USV (persistent surface presence, RF + acoustic relay), and UUV (slow, comms-constrained, acoustic-only) can cooperate — e.g., USV as an acoustic-to-RF communications relay for the UUV
- Latency and bandwidth modeling between domains — this is often the actual engineering bottleneck in real programs (e.g., DARPA-style ACTUV/AUV cooperative concepts)
- **Lab:** Combine your three Week-17 sims into one scenario: UAV surveys an area and cues a target; USV transits to relay position; UUV navigates (DVL + periodic USBL fix relayed via USV) to inspect the cued target

### Week 19: Verification, Validation & Monte Carlo Across Domains
- Dispersion/Monte Carlo analysis adapted to this domain: current/wind variability, sensor dropout probability, comms outage duration distributions
- Requirements-driven design (e.g., "UUV must reach target within X m of cued position despite Y% comms-outage probability") — practice writing and testing against a requirement, not just running a simulation
- **Lab:** Run a Monte Carlo campaign on your Week 18 heterogeneous mission varying current, comms latency/outage, and sensor noise; produce mission-success-rate statistics

### Week 20: Case Studies
Read and analyze (unclassified/public sources):
- **DARPA Sea Hunter / ACTUV** — autonomous USV, COLREGS-compliant long-endurance autonomy
- **REMUS / Bluefin AUVs** — DVL/INS navigation approach, mine-countermeasure and survey missions
- **Saildrone** — long-endurance USV, wind/wave propulsion, ocean survey autonomy
- **Slocum/Seaglider underwater gliders** — buoyancy-driven low-power long-endurance navigation
- **Group 5 UAVs (Global Hawk/Reaper class) and small-UAS swarm autonomy programs** (e.g., DARPA OFFSET) for the air-domain side
- **Deliverable:** 3–5 page memo comparing GNC architecture choices across two of these systems (e.g., "why does Sea Hunter's guidance differ from REMUS's?")

---

## MONTH 6 — Capstone Project & Career Preparation

### Weeks 21–23: Capstone Project
Recommended capstone (builds directly on your Month 5 work):

**"Cooperative Search-and-Inspect Mission"** — a UAV performs wide-area search and target cueing; a USV transits to a relay/overwatch position and provides a communications bridge; a UUV, receiving intermittent cued position updates via the USV, autonomously navigates (DVL dead-reckoning + periodic USBL correction) to close in on and "inspect" (arrive within tolerance of) the target — all under currents, wind, and realistic comms latency/outage.

**Required components:**
- Three working 6-DOF (or reduced-DOF, clearly justified) vehicle plants
- Domain-appropriate navigation filters for each vehicle (including at least one GPS-denied/degraded case)
- LOS or RRT*-based guidance with a COLREGS-compliant avoidance behavior on the USV
- Backstepping or sliding-mode control on the underactuated vehicles; cascaded autopilot on the UAV
- Comms/latency model between vehicles
- Monte Carlo verification against a stated mission-success requirement
- Clean GitHub repo + a 10–15 page report (AIAA/IEEE OES conference-paper style) + a 15–20 slide review deck

**Alternative capstone options** if you want to specialize narrower:
- Pure UUV: full DVL/USBL/terrain-relative navigation stack + fuel/energy-optimal path planning for a long-duration survey mission
- Pure USV: COLREGS-compliant autonomous transit through simulated shipping-lane traffic with Monte Carlo collision-risk analysis
- Pure UAV: GPS-denied VIO/SLAM-based indoor or urban-canyon navigation with obstacle-aware RRT* replanning

### Week 24: Portfolio & Documentation Polish
- Organize all code into a GitHub portfolio; make sure the README for each project explains the domain-specific physics (a reviewer from a UAV-only company may not know what a DVL is — write for that reader)
- Produce 2–3 strong visualizations: multi-vehicle mission animation, Monte Carlo mission-success heat maps, cross-track error / miss-distance plots

### Week 25: Technical Interview Preparation
- Whiteboard-ready derivations: LOS guidance geometry, EKF predict/update, Fossen's underactuated marine vehicle equations, quadrotor cascaded control structure, backstepping control derivation for a simple 1-DOF system
- Practice explaining trade-offs specific to this track: DVL/USBL vs. GPS-denied VIO, LOS vs. pure-pursuit guidance, backstepping vs. sliding mode, COLREGS-compliant vs. generic velocity-obstacle avoidance, RF vs. acoustic comms constraints
- Research your target companies specifically — a defense-maritime-autonomy company, a commercial ocean-survey company, and a UAV-swarm-autonomy startup will each probe very different depths of this material; tailor your capstone narrative

### Week 26: Applications & Final Review
- Resume bullets quantified around your projects (e.g., "Designed backstepping USV controller achieving <X m cross-track error under Y kn current in Monte Carlo trials across 500 dispersed runs")
- Apply, iterate based on interview feedback

---

## Assessment Plan (Self-Graded Milestones)
| Milestone | Week Due | What "Done" Looks Like |
|---|---|---|
| M1a/M1b: USV + UAV + UUV plants | 4 | All three sims run, physically plausible responses, sanity-checked against known trim/step behavior |
| M2a: DVL/USBL EKF | 7 | Bounded position error with periodic fixes; unbounded drift demonstrated without them |
| M3a: LOS + COLREGS avoidance | 11 | USV follows path, correctly gives way per COLREGS in a crossing scenario |
| M4a: Backstepping USV control | 14 | Outperforms LQR baseline under disturbance, no instability |
| M5a: Heterogeneous mission sim | 18 | All three vehicles complete a cooperative mission end-to-end in one simulation |
| Capstone | 23 | Full report + code + slide deck, Monte Carlo verification vs. a stated requirement |

---

## A Few Honest Notes for This Track
- The single hardest conceptual shift from generic aerospace GNC is **underactuation** — you cannot always command "whatever motion you want" the way you can with a fully-actuated spacecraft. Backstepping and sliding-mode control exist largely because of this; spend real time on Week 14.
- COLREGS is a genuinely distinctive, maritime-only requirement. It's a favorite interview differentiator — most generalist GNC candidates have never heard of it, and being fluent in it signals real domain knowledge.
- Underwater comms constraints (acoustic modems: seconds of latency, kbps bandwidth) are the thing that most surprises engineers coming from an aerospace/UAV background. Multi-vehicle GNC design for UUV-involving teams is dominated by "what can you even communicate, and when" — internalize this early, it shapes every architecture decision in Month 5–6.
- If your target companies lean defense (Sea Hunter/ACTUV-style programs), emphasize COLREGS + multi-domain coordination. If they lean commercial ocean survey (Saildrone-style, offshore energy inspection), emphasize long-endurance navigation accuracy and energy-optimal guidance. If they lean UAV/drone autonomy, emphasize VIO/SLAM and cascaded autopilot design. Know which door you're walking through and weight your capstone accordingly.
