reset

set title 'velocity - delay 0.1 sec (mavlink)'
set xlabel 'loop times'
set ylabel 'Height'

set term png enhance font 'Times_New_Roman,12'

set output 'ft_ex_tv.png'

set xtic 10
set ytic 5

set xtics rotate by 45 right

plot [0:100][90:120] "ex0.txt"using 0:2 with points title '0.25 m/s', \
"ex1.txt"using 0:2 with points title '0.2 m/s', \
"ex2.txt"using 0:2 with points title '0.15 m/s', \
"ex3.txt"using 0:2 with points title '0.1 m/s', \
