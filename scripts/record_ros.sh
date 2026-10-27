#!/bin/bash
# Record the SoT controller's task errors to results/ros_<scenario>.csv.
# Start the controller first (other terminal):  ros2 launch sot_ros sot.launch.py scenario:=B
# Then:                                          bash scripts/record_ros.sh B
# Columns: offset(0), t, e_pos [m], e_ori [rad], e_elb [m], e_post [rad], leak
SCENARIO=${1:-A}
SECONDS_TO_RECORD=${2:-14}
cd "$(dirname "$0")/.." && mkdir -p results
echo "Recording /sot_controller/task_errors for ${SECONDS_TO_RECORD} s -> results/ros_${SCENARIO}.csv"
timeout "${SECONDS_TO_RECORD}" ros2 topic echo --csv /sot_controller/task_errors \
  std_msgs/msg/Float64MultiArray > "results/ros_${SCENARIO}.csv"
echo "Done: $(wc -l < "results/ros_${SCENARIO}.csv") samples"
