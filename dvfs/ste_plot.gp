reset

set title 'stereo distance'
set xlabel 'real distance (cm)'
set ylabel 'stereo distance (cm)'

set term png enhance font 'Times_New_Roman,10'

set output 'ste_dis.png'

set xtic 10
set ytic 100

set xtics rotate by 45 right

plot [0:200][0:1000] "ste_dis_25.txt"using 1:2 with points title '25cm', \
"ste_dis_50.txt"using 1:2 with points title '50cm', \
"ste_dis_75.txt"using 1:2 with points title '75cm', \
"ste_dis_100.txt"using 1:2 with points title '100cm', \
"ste_dis_125.txt"using 1:2 with points title '125cm', \
"ste_dis_150.txt"using 1:2 with points title '150cm', \
"ste_dis_175.txt"using 1:2 with points title '175cm', \
"bb_dis_200.txt"using 1:2 with points title '200cm', \
