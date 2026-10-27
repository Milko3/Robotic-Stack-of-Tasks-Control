#!/bin/bash
# Copy the Panda 3D models (meshes) from the course lab repo into ros2/sot_ros/meshes,
# so RViz can draw the robot. Run once, in Git Bash on your own computer, then commit.
#   bash scripts/copy_meshes.sh                       # default: lab repo next to project/
#   bash scripts/copy_meshes.sh /path/to/franka_gazebo/meshes
cd "$(dirname "$0")/.."
SRC=${1:-../../../labs/robotteknik/workspace/src/robot_models/franka_gazebo/meshes}
if [ ! -d "$SRC/visual" ]; then
  echo "Could not find $SRC/visual. Pass the path to franka_gazebo/meshes as an argument."
  exit 1
fi
mkdir -p ros2/sot_ros/meshes
cp -r "$SRC/visual" "$SRC/collision" ros2/sot_ros/meshes/
echo "Copied meshes to ros2/sot_ros/meshes:"
ls ros2/sot_ros/meshes/visual
