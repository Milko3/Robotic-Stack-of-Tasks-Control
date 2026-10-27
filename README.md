# Robotic Stack-of-Tasks Control

Stack-of-Tasks (prioritized task-space control) for the 7-DoF Franka Panda, in C++ with Pinocchio.
Course project, Introduction to Robotics (DT512A).

## Setup (once)

Requires Docker Desktop.

```bash
git clone https://github.com/Milko3/Robotic-Stack-of-Tasks-Control.git
cd Robotic-Stack-of-Tasks-Control/docker
bash build.sh
```

## Build and run

Start the container (from the `docker/` folder):

```bash
bash run.sh
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /sot/build && cd /sot/build
cmake .. && make
./sot
```

After editing code, from `/sot/build`: `make && ./sot`

The repo is mounted at `/sot` inside the container, so edits on your own computer show up immediately.
Open a second terminal into the same container with `bash attach.sh`.

## Code layout and ownership

| File | Owner | What |
|---|---|---|
| `include/sot_types.hpp` | **shared** | `Task` and `Scenario` structs: the contract between all parts |
| `include/sot_core.hpp`, `src/sot_core.cpp` | **A** | `dpinv`, `nullspaceProjector`, `sotStep`, `flatStep` |
| `include/robot.hpp`, `src/robot.cpp` | **B** | Load Panda, forward kinematics, frame Jacobians |
| `include/tasks.hpp`, `src/tasks.cpp` | **B** | Tasks: EE position, EE orientation, elbow height, posture |
| `src/main.cpp` | **C** | Simulation loop, scenarios A / B / B_flat, logging, later ROS 2 control |
| `models/panda.urdf` | shared | Panda arm (no hand), generated from the course `franka_gazebo` xacro |
| `docker/` | shared | ROS 2 Humble + Pinocchio environment |

Python reference for everything: `step1_prototype/sot_panda.py` and `panda_kin.py`.
Pinocchio's kinematics have been checked against `panda_kin.py`: same EE position, elbow height and Jacobian at `Q_READY`.

Functions marked `TODO (A)`, `TODO (B)`, `TODO (C)` are stubs that compile but return zeros.

## Git rules

- Work on your own branch from `start1`, e.g. `core` (A), `tasks` (B), `sim` (C).
- Only edit the files you own.
- Changing `sot_types.hpp`, a header signature, or `CMakeLists.txt`? Tell the group first.
- `git pull origin start1` often; merge back with small pull requests.
