reset

set title 'performance (not see object)'
set xlabel 'time (sec)'
set ylabel 'seconds per frame'

set term png enhance font 'Times_New_Roman,10'

set output 'nonobject.png'

set xtic 10

set xtics rotate by 45 right

plot [0:100][0:0.1] \
"1core.txt"using 1:2 with linespoints title '1 core', \
"2core.txt"using 1:2 with linespoints title '2 core', \
"3core.txt"using 1:2 with linespoints title '3 core', \
"4core.txt"using 1:2 with linespoints title '4 core', \
"5core.txt"using 1:2 with linespoints title '5 core', \
"6core.txt"using 1:2 with linespoints title '6 core', \
