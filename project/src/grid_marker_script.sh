#!/bin/bash
overall_string=''


grid_size='1.2'
count_offset='100'


for x in 0 1 2 3 4 5 6 7

do
	for y in 0 1 2 3 4 5 6 7

	do

	count=$(($x*3 + $y + $count_offset))

	xpos=$(expr $grid_size*$x | bc)
	ypos=$(expr $grid_size*$y | bc)

	#string=$count$' '$xpos$' '$ypos
	#echo $string

	cmd_lin=$count$'\n'$xpos$'\n'$ypos$'\n'
	#echo -ne $cmd_lin

	overall_string=$overall_string$cmd_lin

	done

done

overall_string=$overall_string$'-1\n'
echo -ne $overall_string

echo -ne "$overall_string" | rosrun ar_track_alvar createMarker -s 0.75 -p