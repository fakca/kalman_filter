#!/bin/bash
gnuplot <<- EOF
set term png
set output "build/plot.png"
set xlabel "X"
set ylabel "Y"
plot "build/data.txt" u 1:2 w lines title "Estimation", \
     "build/data.txt" u 3:4 w points pt 7 ps 2 title "Measurement"
EOF
