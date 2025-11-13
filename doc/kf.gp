# 使用 ARG0, ARG1, ARG2... 接收命令行参数
# ARG0 是脚本名,ARG1是第一个参数
data = (exists("ARG1")&& strlen(ARG1)) ? ARG1 : 'kf.txt'
pic = (exists("ARG2")&& strlen(ARG2)) ? ARG2 : 'kf.png'
print data
print pic
set terminal pngcairo size 1920,1080 enhanced font 'Arial,12'
set output pic

set title 'Kalman Filter - Position Estimation'
set xlabel 'Time (s)'
set ylabel 'Position (m)'
set grid
set key top left

plot data using 1:2 with lines lw 2 title 'True Position', \
     '' using 1:3 with points pt 7 ps 0.5 title 'Measured Position', \
     '' using 1:4 with lines lw 2 title 'Estimated Position'

unset output