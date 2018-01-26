#!/usr/bin/env gnuplot

set term pngcairo enhanced size 800,600 font 'Hack'
set datafile separator ","

#set yrange [-1.75:1.75]
set xrange [0: 600]

set autoscale y
set autoscale y2
set y2tics nomirror
set ytics nomirror

set ylabel 'cte'
set y2label 'epsi'
set output 'objective.png'
set title 'Optimization objective'
plot 'drive.csv' using 1:3 with lines title 'cte' lw 2 axes x1y1, \
     ''          using 1:4 with lines title 'epsi' lw 2 axes x1y2
     

set y2label 'steeting'
set ylabel 'acceleration'
set output 'actuators.png'
set title 'Actuators'
plot 'drive.csv' using 1:7 with lines title 'acceleration' lw 2 axes x1y1, \
     ''          using 1:8 with lines title 'steering' lw 2 axes x1y2

unset y2tics
unset y2label
set ytics mirror
set ylabel 'velocity'
set output 'velocity.png'
set title 'Velocity'
plot 'drive.csv' using 1:5 with lines title 'velocity' lw 2
