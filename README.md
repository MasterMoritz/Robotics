# Robotics

Task:
 - Create an Android application which provides mobile robot navigation to the desired predefined goal pose (x,y,theta)
 from pose (0,0,0) while avoiding obstacles along the way.

TODO:

implement bug 2 algorithm:

 - fix followObstacle method:
	- mline encounters have to be tested yet
	- driveUntil is unfortunately too inaccurate for whatever reasons, thus mline check is hardcoded
	- only works if left sensor sees the obstacle at beginning atm
