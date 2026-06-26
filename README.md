# AUVSimBench

A lightweight **physics twin** for autonomous underwater vehicles — a 6-DOF AUV
simulator as a single ROS 2 node, in pure Python/numpy. It runs the vehicle's
physics and publishes the **sensor topics a state estimator consumes, plus
ground-truth pose**, so you can develop and stress-test an underwater localization
or control stack **on the bench** — no water, no GPU, no Isaac Sim. It runs happily
on a Jetson.

## Why this is hard (and why the twin helps)

Validating underwater state estimation is uniquely painful:

- **Wet tests are expensive and rare.** Pool/ocean time is slow to schedule, and you
  get only a handful of runs before everyone's tired and the battery's flat.
- **There is no ground truth underwater.** No GPS below the surface — so even when you
  *do* get a dive, you often can't measure *how wrong* the estimate was.
- **Most failures aren't about the water.** In practice the things that break a dive are
  wiring (a node subscribed to the wrong topic), frame conventions (axes flipped),
  startup ordering (a sensor not powered on), or an estimator diverging — and you
  usually discover them *mid-dive*, burning a wet test to debug a typo.
- **Recorded bags only go so far.** A bag replays exactly one motion; you can't steer
  it, you can't make a sensor fail on cue, and replayed DVL/depth won't agree with a
  real IMU sitting still on the bench.

A **physics twin** answers all four. It simulates the vehicle's dynamics, so every
sensor stream is derived from *one consistent truth*; it *knows* the true pose, so
estimator accuracy is measurable; and you can script scenarios, disturbances, and
on-demand sensor failures — all from a laptop or the robot's own compute.

```
wrench in  ──►  6-DOF dynamics (thrust + buoyancy + drag + current)  ──►  IMU / DVL / depth + ground truth
```

## What it gives you

- **Consistent sensors** — IMU, DVL and depth all generated from the same simulated
  body, so they physically agree (the thing bag-replay can't do).
- **Ground truth** — `/auv_sim/ground_truth` is the exact pose, so you can plot
  estimator error directly.
- **Scenarios & disturbances** — water current, and on-demand sensor failures (DVL
  dropout, noise, NaN) to probe estimator robustness — the exact failure modes that
  bite in real wet tests.
- **Driven by your real controller** — it takes a `geometry_msgs/Wrench` with a deadman,
  the same contract a PID/thruster-allocation stack already produces.

## Physics model

An independent **6-DOF Newton-Euler** rigid body (FLU body frame, ENU world,
quaternion orientation):

- **Buoyancy** `ρ·V·g`, applied at a **center of buoyancy above the center of mass**, so
  the vehicle self-rights — giving realistic roll/pitch, and therefore realistic IMU
  output. Default displacement is exactly neutral.
- **Linear + quadratic drag** on both linear and angular velocity.
- **Water current** — drag acts on velocity *relative to the water*, so a current
  actually pushes a passive vehicle (modeled properly, not faked through gravity).
- **Wrench input** in the body frame, with a 0.5 s command-timeout deadman.

The accelerometer output is the true specific force `(ΣF − gravity)/m` in the body
frame, so a level, neutrally-buoyant vehicle reads `az ≈ +9.81` — matching a real
ENU/FLU IMU.

## Vehicle-agnostic

It is **not tied to any one vehicle.** Every topic name, frame, rate and physics
constant is a parameter, and the defaults are generic:

| published topic (default) | type | content |
|---------------------------|------|---------|
| `imu/data` | `sensor_msgs/Imu` | orientation, body angular velocity, specific-force accel |
| `dvl/odometry` | `nav_msgs/Odometry` | body-frame velocity |
| `dvl/altitude`, `ping/range` | `sensor_msgs/Range` | height above the floor |
| `pressure` | `sensor_msgs/FluidPressure` | from depth |
| `ground_truth` | `geometry_msgs/PoseStamped` | exact sim pose |

Adapt it to a specific vehicle with a **profile** — a small YAML that remaps the
topics/frames and sets the mass/buoyancy. `config/barracuda.yaml` is one example
(Barracuda's real `/barracuda/...` topics); write your own for any other AUV.

## See it in 3D

There's no built-in render, but it broadcasts **TF** and a **vehicle marker**, so it
shows up live in **RViz2** — which runs on a Jetson (plain OpenGL, no RTX/Isaac):

```bash
ros2 launch auv_sim_bench sim.launch.py rviz:=true
```

You get a 3D box for the vehicle flying around under thrust/current/buoyancy, the TF
frames, and — if you set `estimate_topic` to your estimator's pose — a second marker
for the **estimate**, so you can watch estimate-vs-truth drift in real time. It's
schematic (boxes + axes), not photoreal; photoreal water is the one thing that needs
Isaac Sim on an x86 box.

## Failure & disturbance injection

For the real reason to simulate — breaking the estimator on purpose:

```yaml
current: [0.3, 0.0, 0.0]        # steady cross-current
dvl_dropout_start_sec: 10.0     # DVL goes silent at t=10s ...
dvl_dropout_duration_sec: 5.0   # ... for 5s
dvl_noise_stddev: 0.02          # velocity noise
dvl_emit_nan: false             # or feed the estimator NaNs
```

## Build & run

```bash
cd ~/ros2_ws/src && git clone <this repo> auv_sim_bench
cd ~/ros2_ws && colcon build --packages-select auv_sim_bench
source install/setup.bash

ros2 launch auv_sim_bench sim.launch.py                       # generic defaults
ros2 launch auv_sim_bench sim.launch.py rviz:=true            # + 3D view
ros2 launch auv_sim_bench sim.launch.py config:=barracuda.yaml   # Barracuda profile
# drive it by publishing geometry_msgs/Wrench on the wrench topic (default cmd_wrench)
```

### On a Jetson (AGX Orin)

Validated on a Jetson AGX Orin (JetPack 6 / ROS 2 Humble) — pure Python + numpy, so
no GPU or Isaac needed:

```bash
mkdir -p ~/auv_ws/src
cp -r AUVSimBench ~/auv_ws/src/auv_sim_bench
cd ~/auv_ws && colcon build --packages-select auv_sim_bench
source /opt/ros/humble/setup.bash
source ~/auv_ws/install/setup.bash        # both source lines, in every new terminal
ros2 launch auv_sim_bench sim.launch.py
```

Headless over SSH there's no display for RViz, so drop `rviz:=true` and watch the
data instead (`ros2 topic echo /ground_truth`); driving it with a `/cmd_wrench`
publisher moves the body as expected. For the 3D view, run RViz on a monitor
attached to the Jetson.

Test your estimator: run the sim + your estimator, then compare
`/barracuda/estimated_pose` against `/auv_sim/ground_truth` — an accuracy check you
cannot get from a real dive (no underwater ground truth) or from bag replay.

## Tests

The physics core (`dynamics.py`) is plain numpy with no ROS dependency, so it is
unit-tested directly — buoyancy, gravity, drag, thrust response, and current:

```bash
pytest test/
```

## Credits & scope

Inspired by Leonardo Lima's [`isaac_underwater`](https://github.com/leonardomdl/isaac_underwater)
examples (MIT), which show buoyancy and drag in Isaac Sim. AUVSimBench takes that
idea in a different direction — its own 6-DOF dynamics in a standalone ROS 2 node, so
it runs anywhere ROS 2 does, including a Jetson.

It's a controls/estimation test rig, not a hydrodynamics research tool: drag and
inertia are simple tunable coefficients, not CFD. Added-mass and richer current
fields are natural next steps.
