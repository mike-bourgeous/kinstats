all:
	gcc -g -O2 -Wall kinstats.c -o kinstats -lfreenect

debug:
	gcc -g -O0 -Wall kinstats.c -o kinstats -lfreenect


