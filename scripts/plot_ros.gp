# plot_ros.gp - errors recorded from the ROS 2 controller (scripts/record_ros.sh test).
# Run from the repo root:  gnuplot scripts/plot_ros.gp   -> results/ros_errors.png
# Columns in results/ros_test.csv: 1 offset, 2 t [s], 3 yaw error [rad], 4 position error [m]

set datafile separator ','
f = 'results/ros_test.csv'
if (strstrt(GPVAL_TERMINALS, 'pngcairo') > 0) {
    set terminal pngcairo size 900,500 font ',11'
} else {
    set terminal png size 900,500
}
set output 'results/ros_errors.png'
set title 'Task errors on the Panda via ros2\_control (log scale)'
set logscale y
set format y '10^{%L}'
set xlabel 'time since controller start [s]'
set ylabel 'error'
set grid
set key top right opaque
plot f using 2:(1e-12 + abs($3)) with lines lw 2 title 'task 1: yaw error [rad]', \
     f using 2:(1e-12 + $4) with lines lw 2 title 'task 2: position error [m]'
print 'Saved results/ros_errors.png'
