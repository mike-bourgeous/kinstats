#!/bin/bash
# Controls lights based on average distance of scene from Kinect camera.
# Object 13 parameter 4 is the light value
# Created Jan. 7, 2011 by Mike Bourgeous

./kinstats -a | \
	grep --line-buffered '^[0-9]*$' | \
	awk -W interactive '{ print "set 13,4," (1000 - $0) / 2 }' | \
	tee /dev/stderr | \
	nc logic-controller.local 14309 | \
	grep --line-buffered -v OK


