reset

set title 'power (not see object)'
set xlabel 'frame'
set ylabel 'seconds per frame'

set term png enhance font 'Times_New_Roman,10'

set output 'nonobject_power.png'

set xtic 10

set xtics rotate by 45 right

plot [0:100][0:5] \
"2core_power.txt"using 1:2 with linespoints title '2 core', \
"6core_power.txt"using 1:2 with linespoints title '6 core', \
