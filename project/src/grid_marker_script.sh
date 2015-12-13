#!/bin/bash
overall_string=''


grid_size='0.70'
count_offset='0'


#don't forget to change the number on line 17 to be the maximum number on line 10~ PLUS ONE!
#YOu'll waste a lot of time if you don't do this!
for x in 0 1 2 3 4 5 6 7

do
	for y in 1 2 3 4 5 6 7 8

	do

	count=$(($x*8 + $y + $count_offset))

	echo $count

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
#echo -ne $overall_string

echo -ne "$overall_string" | rosrun ar_track_alvar createMarker -uin -s 0.619 -p