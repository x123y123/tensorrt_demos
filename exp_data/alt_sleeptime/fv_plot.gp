reset

set title 'velocity 0.1 m/s - delay (mavlink)'
set xlabel 'loop times'
set ylabel 'Height'

set term png enhance font 'Times_New_Roman,12'

set output 'fv_ex_tv.png'

set xtic 10
set ytic 5

set xtics rotate by 45 right

plot [0:100][90:110] "fv_ex0.txt"using 0:2 with points title 'sleep(0.01)', \
"fv_ex2.txt"using 0:2 with points title 'sleep(0.05)', \
"fv_ex1.txt"using 0:2 with points title 'sleep(0.1)', \
"fv_ex3.txt"using 0:2 with points title 'sleep(0.5)', \
