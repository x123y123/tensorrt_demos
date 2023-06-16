reset

set title 'drone path'
set xlabel 'longitude' offset 0,-2 font 'Times_New_Roman, 12'
set ylabel 'latitude'  offset 2,-2 font 'Times_New_Roman, 12'
set zlabel 'altitude'  rotate by 90 right font 'Times_New_Roman, 12'

set term png enhance font 'Times_New_Roman,12'

#set output 'path.png'
set output 'path1.png'


set xtic 0.00001 offset 0,-1
set ytic 0.00001 
set ztic 1 
set ticslevel 0
set xtics rotate by 90 right font 'Times_New_Roman, 10'
set ytics rotate by 45 right font 'Times_New_Roman, 10'
set ztics rotate by 45 right font 'Times_New_Roman, 10'

splot [120.47767:120.4777][23.56263:23.56267][104:107] \
"path_data_1.txt"using 5:4:6 with lines lw 2 title 'path', \
#splot [120.4777:120.47771][23.562695:23.5627][110:120] \
#"path_data.txt"using 5:4:6 with lines lw 2 title 'path', \
