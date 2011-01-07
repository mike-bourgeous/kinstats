/*
 * Test Kinect/libfreenect program to display some depth stats.
 * Created Jan. 6, 2011
 * (C)2011 Mike Bourgeous
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <math.h>

#include <libfreenect/libfreenect.h>


#define INFO_OUT(...) {\
	printf("%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	printf(__VA_ARGS__);\
}
#define ERROR_OUT(...) {\
	fprintf(stderr, "\e[0;1m%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	fprintf(stderr, __VA_ARGS__);\
	fprintf(stderr, "\e[0m");\
}

#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array)[0]))


#define SM_HIST_SIZE	32

// Get a depth pixel from an 11-bit buffer stored in uint16_t
#define DPT(buf, x, y) (buf[(x) * FREENECT_FRAME_W + (y)])

// Convert pixel number to coordinates
#define PX_TO_X(pix) (pix % FREENECT_FRAME_W)
#define PX_TO_Y(pix) (pix / FREENECT_FRAME_W)

// Depth gamma look-up table (I wish freenect provided a user data struct for callbacks)
static float depth_lut[2048];
static int out_of_range = 0;

void repeat_char(int c, int count)
{
	int i;
	for(i = 0; i < count; i++) {
		putchar(c);
	}
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	uint16_t *buf = (uint16_t *)depthbuf;
	int big_histogram[2048], small_histogram[SM_HIST_SIZE];
	uint16_t min, max;
	int min_pix, max_pix;
	int total = 0;
	int oor_count = 0; // Out of range count
	int median;
	int i;

	memset(big_histogram, 0, sizeof(big_histogram));
	memset(small_histogram, 0, sizeof(small_histogram));

	min = UINT16_MAX;
	max = 0;
	min_pix = -1;
	max_pix = -1;
	for(i = 0; i < FREENECT_FRAME_PIX; i++) {
		if(buf[i] == 2047) {
			oor_count++;
			continue;
		}

		big_histogram[buf[i]]++;
		small_histogram[buf[i] * SM_HIST_SIZE / 2048]++;

		if(buf[i] < min) {
			min = buf[i];
			min_pix = i;
		}
		if(buf[i] > max) {
			max = buf[i];
			max_pix = i;
		}
		total += buf[i];
	}

	for(median = 0, i = 0; i < 2048 && median < (FREENECT_FRAME_PIX - oor_count) / 2; i++) {
		median += big_histogram[i];
	}
	median = i;

	// Clear the screen
	printf("\e[H\e[2J");

	INFO_OUT("Time: %u, min: %hu (%d, %d), max: %hu (%d, %d)\n",
			timestamp,
			min, PX_TO_X(min_pix), PX_TO_Y(min_pix),
			max, PX_TO_X(max_pix), PX_TO_Y(max_pix));

	INFO_OUT("Out of range: %d%% mean: %f (%f), median: %d (%f)\n",
			oor_count * 100 / FREENECT_FRAME_PIX,
			(double)total / (double)FREENECT_FRAME_PIX,
			depth_lut[(int)((double)total / (double)FREENECT_FRAME_PIX)],
			median, depth_lut[median]);

	for(i = 0; i < SM_HIST_SIZE; i++) {
		printf("%*.4f: ", 9, depth_lut[i * 2048 / SM_HIST_SIZE]);
		repeat_char(i == median * SM_HIST_SIZE / 2048 ? '*' : '-',
				small_histogram[i] * 96 / FREENECT_FRAME_PIX);
		printf("\n");
	}
	printf("%*s: ", 9, "Out");
	repeat_char('-', oor_count * 96 / FREENECT_FRAME_PIX);
	printf("\n");

	// Make LED red if more than 35% of the image is out of range
	out_of_range = oor_count > FREENECT_FRAME_PIX * 35 / 100;
}


static int done = 0;

void intr(int signum)
{
	INFO_OUT("Exiting due to signal %d (%s)\n", signum, strsignal(signum));
	done = 1;

	signal(signum, exit);
}

// http://groups.google.com/group/openkinect/browse_thread/thread/31351846fd33c78/e98a94ac605b9f21#e98a94ac605b9f21
void init_lut()
{
	int i;

	for(i = 0; i < 2048; i++) {
		depth_lut[i] = 0.1236 * tanf(i / 2842.5 + 1.1863);
		printf("%d: %f\n", i, depth_lut[i]);
	}
}

int main()
{
	freenect_context *kn;
	freenect_device *kn_dev;

	if(signal(SIGINT, intr) == SIG_ERR ||
			signal(SIGTERM, intr) == SIG_ERR) {
		ERROR_OUT("Error setting signal handlers\n");
		return -1;
	}

	init_lut();

	if(freenect_init(&kn, NULL) < 0) {
		ERROR_OUT("libfreenect init failed.\n");
		return -1;
	}

	INFO_OUT("Found %d Kinect devices.\n", freenect_num_devices(kn));

	if(freenect_num_devices(kn) == 0) {
		ERROR_OUT("No Kinect devices present.\n");
		return -1;
	}

	if(freenect_open_device(kn, &kn_dev, 0)) {
		ERROR_OUT("Error opening Kinect #0.\n");
		return -1;
	}

	freenect_set_tilt_degs(kn_dev, 15);
	freenect_set_led(kn_dev, LED_GREEN);
	freenect_set_depth_callback(kn_dev, depth);
	freenect_set_depth_format(kn_dev, FREENECT_DEPTH_11BIT);

	freenect_start_depth(kn_dev);

	int last_oor = out_of_range;
	while(!done && freenect_process_events(kn) >= 0) {
		if(last_oor != out_of_range) {
			freenect_set_led(kn_dev, out_of_range ? LED_BLINK_RED_YELLOW : LED_GREEN);
			last_oor = out_of_range;
		}
	}

	freenect_stop_depth(kn_dev);
	freenect_set_led(kn_dev, LED_OFF);
	freenect_close_device(kn_dev);
	freenect_shutdown(kn);

	return 0;
}
