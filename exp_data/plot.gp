reset

set title 'bounding box area to distance'
set xlabel 'distance (cm)'
set ylabel 'Area (pix*pix)'

set term png enhance font 'Times_New_Roman,10'

set output 'area2dis.png'

set xtic 50

set xtics rotate by 45 right

plot [0:450][0:16000] "bb_dis_50.txt"using 1:2 with points title '50cm', \
"bb_dis_100.txt"using 1:2 with points title '100cm', \
"bb_dis_150.txt"using 1:2 with points title '150cm', \
"bb_dis_200.txt"using 1:2 with points title '200cm', \
"bb_dis_250.txt"using 1:2 with points title '250cm', \
"bb_dis_300.txt"using 1:2 with points title '300cm', \
"bb_dis_350.txt"using 1:2 with points title '350cm', \
"bb_dis_400.txt"using 1:2 with points title '400cm', \
"bb_dis_total.txt"using 1:2 with linespoints title 'total', \
