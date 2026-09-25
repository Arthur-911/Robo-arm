# 🦾 Robo-Arm (kine-rs)

> An interactive 3D robotic arm simulator and kinematics engine built from scratch in pure Rust. Runs blazing fast on your desktop and in WebAssembly.

---

## 👋 Hey there! Welcome to Robo-Arm

If you've ever worked with industrial robotic arms, you know that setting up heavy frameworks like ROS / MoveIt just to test an inverse kinematics algorithm or visualize a trajectory can be a headache. 

**Robo-Arm** was built to solve that. It is a lightweight, zero-bloat, pure Rust robotics simulator that launches in under a second and gives you an interactive 3D workspace with real-time inverse kinematics, physics-inspired manipulation, smooth motion profiles, and an AI-powered conversational command console.

Whether you want to test a 6-DOF industrial robot, design a trajectory, import a custom URDF, or just command the arm using plain English ("wave", "pick up the red cube", "joint 1 45°"), this tool has you covered.

---

## ⚡ Quick Start: Running the Simulator

All you need is the standard Rust toolchain ([rustup.rs](https://rustup.rs/)).

```bash
# 1. Clone the repository
git clone https://github.com/Arthur-911/Robo-arm.git
cd Robo-arm

# 2. Run tests to make sure everything is green
cargo test

# 3. Launch the desktop simulator
cargo run --release --bin kine-sim
```

That's it! In a couple of seconds, the 3D interactive simulator window will open on your screen.

---

## 🎮 How to Control the Arm

We designed the controls so you can interact with the robot in whatever way feels most natural:

### 1. 🖱️ Direct 3D Canvas Gizmos
- **Translate Target**: Left-click and drag the red (+X), green (+Y), or blue (+Z) axes on the tool center point (TCP).
- **Free View-Plane Drag**: Drag the white center sphere to have the robot hand track your mouse pointer directly in 3D space.
- **Rotate Orientation**: Drag the colored orientation rings (Roll, Pitch, Yaw) to rotate the end-effector.
- **Camera View**:
  - Right-click and drag to rotate the camera around the robot.
  - Middle-click (or Shift + Left-click) to pan.
  - Mouse wheel to zoom in and out.
  - Press `F` to auto-focus the camera directly on the robot hand.
  - Press `H` to reset the camera to the home view.

### 2. 🦾 Direct Joint Jogging (Robot Tab)
Switch to the **`🤖 Robot`** tab in the sidebar:
- **Jog Buttons**: Each joint has dedicated jog buttons: `[-10°]`, `[-1°]`, `[0°]`, `[+1°]`, and `[+10°]` (or centimeter steps for prismatic joints).
- **Degree Sliders & Numeric Input**: Adjust joints with 0.1° accuracy.
- **Limit Warnings**: An amber `⚠ Limit` indicator lights up if a joint approaches within 3° of its hardware boundary.
- **Pose Shortcuts**: Hit `🏠 Home Position`, `🔄 Zero All Joints`, or `📐 Ready Pose` to quickly reset the posture.
- **Bidirectional Sync**: Moving joints automatically updates the target coordinates so the IK solver never snaps back.

### 3. 💬 AI Command Console (Text Commands)
You can command the robot using plain English or structured text! You can type commands either in the dedicated **`💬 Chat`** tab or in the **Quick Command Bar** anchored at the bottom of the 3D screen:

Here are some real examples you can type right now:

```text
move x 0.4 y 0.2 z 0.3    # Smoothly glides the hand to coordinates (meters)
goto 0.35 0.15 0.25        # Quick coordinate destination
up 0.05                    # Jogs 5 cm upward in world Z
left 0.10                  # Jogs 10 cm to the left
joint 1 45                 # Rotates Joint 1 to 45°
jog j2 +10                 # Jogs Joint 2 by +10°
joints 0, 30, -60, 0, 30, 0# Sets all joints simultaneously
grip                       # Closes the mechanical gripper
release                    # Opens the mechanical gripper
gripper 50%                # Half-opens the gripper jaws
vacuum on                  # Activates suction cup tool
weld on                    # Activates electric arc welding torch
pick red                   # Automatically navigates down, grips red workpiece, and lifts it
wave                       # Plays a friendly robotic wave greeting!
nod                        # Plays a nodding gesture
dance                      # Runs a smooth continuous figure-8 trajectory
home                       # Glides the arm back to the home pose
zero                       # Glides all joints to 0°
status                     # Prints current TCP coordinates, joint angles, and tool status
```

> **Note on Smooth Motion**: Text commands don't teleport the robot in 1 frame. The motion engine uses a continuous **minimum-jerk quintic S-curve** ($10u^3 - 15u^4 + 6u^5$) to smoothly accelerate and decelerate the arm over time, just like a real industrial robot.

### 4. ⌨️ Keyboard Hotkeys
When you're not typing into a text box, you can use these convenient hotkeys:
- `Arrow Keys`: Jog hand Left/Right (X) and Forward/Back (Y)
- `PageUp / PageDown`: Jog hand Up/Down (Z)
- `[` and `]`: Close / Open gripper
- `Space`: Pause or resume trajectory playback
- `G`: Toggle parallel gripper jaws
- `V`: Toggle vacuum suction cup
- `W`: Toggle arc welding torch
- `R`: Reset robot to home configuration
- `C`: Clear the fading motion trail
- `F`: Focus camera on tool center point
- `H`: Reset camera view

---

## 📊 Clean Workspace & Graphing on Demand

We believe the 3D viewport should stay clean and uncluttered. Heavy graphs and torque strain progress bars are **hidden by default** so you can focus on the arm.

Whenever you want to analyze motor dynamics:
- Click the **`📊 Show Dynamics & Strain Graphs`** button in the Robot tab (or toggle `📊 Graphs` in the bottom status bar).
- This opens real-time motor torque readouts (Nm), rated load capacities, and strain percentage bars calculated via recursive Newton-Euler gravity dynamics.
- Click it again anytime to tuck the graphs away and keep your workspace clean.

---

## 🛠️ Workcell Tools & Manipulation

Robo-Arm includes multiple interchangeable End-of-Arm Tooling (EOAT) heads and simulated workpieces:
- **Parallel Mechanical Gripper**: Two articulated fingers that close around objects.
- **Pneumatic Vacuum Cup**: Suction gripper capable of grasping flat objects.
- **Electric Arc Welding Torch**: Welder tip that emits an animated electric arc spark glow.
- **Physics Workpieces**: Includes colored table blocks (Red, Blue, Gold) that can be picked up, carried, and placed.

---

## 📄 Custom Robot URDF Import & Export

Got your own robot arm?
- Simply **drag and drop your `.urdf` file** right into the 3D window.
- Or paste the URDF XML into the **`📄 URDF`** tab editor and click **Load URDF**.
- You can also export any loaded or modified robot configuration back into clean, standard URDF XML with one click.

---

## 📤 Trajectory Exporting

Once you've built waypoints or planned a motion sequence in the **`📈 Traj`** tab, you can export it to production formats:
- 🐍 **Python**: Clean standalone script using `numpy` and `matplotlib` to plot and simulate the trajectory.
- 🤖 **ROS 2**: Ready-to-use `control_msgs/action/FollowJointTrajectory` YAML action goal.
- ⚙️ **CNC G-Code / CSV**: Linear G01/G00 motion instructions and time-series coordinate tables.

---

## 🧠 Under the Hood (How It Works)

For those curious about the engineering details:
- **Mathematics**: Powered by `nalgebra` with zero-allocation cached isometries on links and joints.
- **Forward Kinematics**: Evaluated in $<2\,\mu\text{s}$ using spatial transformations.
- **Inverse Kinematics**:
  - **Jacobian DLS**: Singularity-Robust (SR) Levenberg-Marquardt with adaptive damping based on the Yoshikawa linear manipulability index $\sqrt{\det(J_v J_v^T)}$.
  - **Cholesky Factorization**: Solves $(J J^T + \lambda^2 I) \Delta x = e$ in $O(m^3/3)$ operations rather than costly $O(n^3)$ matrix inversions.
  - **FABRIK**: Geometric heuristic solver with joint angle constraint projection.
- **Collision Checking**: Real-time analytical cylinder/capsule to obstacle distance checks.
- **Smoothness Filter**: Real-time Exponential Moving Average (EMA) and joint velocity clamp smoother to prevent jerky motions.

---

## 📂 Project Structure

```
Robo-arm/
├── src/
│   ├── kinematics/         # Joint transforms, FK/IK solvers (DLS, FABRIK), collisions, dynamics
│   ├── trajectory/         # Minimum-jerk quintic/cubic interpolation, smoother, controller
│   ├── workcell/           # Tool heads (gripper/vacuum/welder), workpieces, obstacles
│   ├── urdf/               # Pure-Rust URDF XML parser and exporter
│   ├── export/             # Code generators for Python, ROS 2, and G-Code
│   ├── presets/            # Built-in models (Industrial 6-DOF, 7-DOF iiwa, SCARA, 2D Planar)
│   ├── ui/                 # eframe/egui interface, 3D orbit camera, and renderer
│   │   ├── chat_command.rs # AI Copilot text command engine & chat console
│   │   ├── app.rs          # Main RoboSimApp loop and smooth motion controller
│   │   └── panels/         # Tab panels (IK, Robot, Trajectory, Workcell, URDF)
│   ├── lib.rs              # Core library exports
│   └── main.rs             # Desktop executable entry point
├── tests/                  # 26 comprehensive integration and unit test suites
├── Cargo.toml              # Rust crate dependencies and build profiles
└── README.md               # You are here!
```

---

## 🤝 Contributing

Contributions, feedback, and feature ideas are welcome! Feel free to open an issue or submit a pull request.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).
