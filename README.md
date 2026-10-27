# Robotic Stack-of-Tasks Control

Equality Stack of Tasks (SoT) for the 7-DoF Franka Panda, in C++ with Pinocchio, run both as a
standalone test and as a **ros2_control** controller shown in RViz.
Course project C2, Introduction to Robotics (DT512A).

## The stack (sot.cpp)

| Priority | Task | Function |
|---|---|---|
| 1 | Hand yaw -> 0 | `standard_method::calculateDq1` |
| 2 | Hand to a goal position (start + offset) | `standard_method::calculateDq2`, projected into task 1's null space |

## Setup (once)

Requires Docker Desktop. For RViz on Windows, also an X server (e.g. VcXsrv), as in the labs.

```bash
bash scripts/copy_meshes.sh     # Git Bash on Windows: Panda 3D models from the lab repo
cd docker && bash build.sh      # again whenever docker/Dockerfile changes
```

## 1. Standalone test

```bash
cd docker && bash run.sh                     # start the container (Git Bash)
# inside the container:
mkdir -p /sot/build && cd /sot/build
cmake .. && make && ./sot                    # prints errors every second, then PASS or FAIL
```

## 2. On the Panda with ros2_control (RViz)

```bash
# inside the container:
cd /sot/ros2 && colcon build && source install/setup.bash
ros2 launch sot_ros sot.launch.py            # add rviz:=false without a display
```

The controller (`ros2/sot_ros/src/sot_controller.cpp`) includes `sot.cpp` and calls
`calculateDq1` and `calculateDq2` every 2 ms. RViz shows the arm, the goal (orange sphere)
and a line from the hand to the goal. The goal offset is set in `ros2/sot_ros/config/controllers.yaml`.

Record and plot a run (second terminal: `bash docker/attach.sh`):

```bash
cd /sot && bash scripts/record_ros.sh test   # while the controller runs -> results/ros_test.csv
gnuplot scripts/plot_ros.gp                  # -> results/ros_errors.png
```

After editing `sot.cpp`: `make` in `/sot/build` for the test, and `colcon build` in `/sot/ros2`
for the controller.
