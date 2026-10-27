# plot.gp - plots of the SoT simulation results.
# Run from the repo root after ./sot:   gnuplot scripts/plot.gp
# Reads  results/scenario_{A,B,B_flat}.csv
# Writes results/errors.png, results/path.png, results/elbow.png

set datafile separator ','
dir = 'results/'
if (strstrt(GPVAL_TERMINALS, 'pngcairo') > 0) {
    set terminal pngcairo size 1500,450 font ',11'
} else {
    set terminal png size 1500,450
}

# ---------------------------------------------------------------- task errors over time
set output dir.'errors.png'
set multiplot layout 1,3 title 'Task error norms (log scale)'
set logscale y
set yrange [1e-8:1e2]
set format y '10^{%L}'
set xlabel 'time [s]'
set grid
set key top right opaque
do for [s in 'A B B_flat'] {
    f = dir.'scenario_'.s.'.csv'
    set title (s eq 'A' ? 'A: all tasks feasible (SoT)' : (s eq 'B' ? 'B: elbow target infeasible (SoT)' : 'B\_flat: same, no priorities'))
    if (s eq 'A') { set ylabel 'error norm' } else { unset ylabel }
    plot f using 1:(1e-12 + column('e_pos')) with lines lw 2 title 'EE position [m]', \
         f using 1:(1e-12 + column('e_ori')) with lines lw 2 title 'EE orientation [rad]', \
         f using 1:(1e-12 + column('e_elb')) with lines lw 2 title 'elbow height [m]'
}
unset multiplot
unset logscale y
unset format y
unset yrange

# ---------------------------------------------------------------- EE path seen from above
if (strstrt(GPVAL_TERMINALS, 'pngcairo') > 0) {
    set terminal pngcairo size 1000,500 font ',11'
} else {
    set terminal png size 1000,500
}
set output dir.'path.png'
set multiplot layout 1,2 title 'End-effector path seen from above (after the first 2 s)'
set size ratio -1
set xlabel 'x [m]'
set ylabel 'y [m]'
do for [s in 'B B_flat'] {
    f = dir.'scenario_'.s.'.csv'
    set title (s eq 'B' ? 'B: SoT keeps the circle' : 'B\_flat: circle is distorted')
    plot f using (column('target_x')):(column('target_y')) with lines dt 2 lc rgb 'black' title 'target circle', \
         f using (column('t') > 2 ? column('ee_x') : NaN):(column('ee_y')) with lines lw 2 lc rgb '#d95f02' title 'end effector'
}
unset multiplot
set size noratio

# ---------------------------------------------------------------- elbow height
set output dir.'elbow.png'
set title 'Elbow (panda\_link4) height'
set xlabel 'time [s]'
set ylabel 'height [m]'
set key top right opaque
plot dir.'scenario_A.csv' using 1:(column('elbow_z')) with lines lw 2 title 'A (target 0.55 m)', \
     dir.'scenario_B.csv' using 1:(column('elbow_z')) with lines lw 2 title 'B (target 0.90 m)', \
     dir.'scenario_B_flat.csv' using 1:(column('elbow_z')) with lines lw 2 title 'B\_flat (target 0.90 m)', \
     0.55 with lines dt 2 lc rgb 'gray' notitle, \
     0.90 with lines dt 2 lc rgb 'gray' notitle

print 'Saved results/errors.png, results/path.png, results/elbow.png'
