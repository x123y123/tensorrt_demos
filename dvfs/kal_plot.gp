reset

set title 'kalman filter'
set xlabel 'time (sec)'
set ylabel 'Area (pix*pix)'

set term png enhance font 'Times_New_Roman,10'

set output 'kalman.png'

#set xtic 100

set xtics rotate by 45 right

plot [0:][0:150] \
"kal_dist_test1.txt"using 0:1 with linespoints title 'origin', \
"kal_dist_test1.txt"using 0:2 with linespoints title 'kalman filter', \
