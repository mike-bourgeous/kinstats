/*
 * Test Kinect/libfreenect program to display some depth stats.
 * Created Jan. 6, 2011
 * (C)2011 Mike Bourgeous
 * Distributed with no warranty under GPLv2 or later.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
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


#define SM_HIST_SIZE	32

// Convert pixel number to coordinates
#define PX_TO_X(pix) (pix % FREENECT_FRAME_W)
#define PX_TO_Y(pix) (pix / FREENECT_FRAME_W)

struct kinstats_info {
	// Depth gamma look-up table
	float depth_lut[2048];

	enum {
		VERBOSE,
		MEDIAN,
		MEDIAN_SCALED,
		AVERAGE,
		AVERAGE_SCALED,
	} disp_mode;

	// Whether >=n% of pixels are out of range
	unsigned int out_of_range:1;

	// Whether to end the main loop
	unsigned int done:1;
};

void repeat_char(int c, int count)
{
	int i;
	for(i = 0; i < count; i++) {
		putchar(c);
	}
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	struct kinstats_info *data = freenect_get_user(kn_dev);
	uint16_t *buf = (uint16_t *)depthbuf;
	int big_histogram[2048], small_histogram[SM_HIST_SIZE];
	uint16_t min, max;
	int min_pix, max_pix;
	int total = 0;
	int oor_count = 0; // Out of range count
	int median;
	float avg;
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

	avg = (double)total / (double)(FREENECT_FRAME_PIX - oor_count);

	switch(data->disp_mode) {
		case VERBOSE:
			// Move to the top of the screen
			printf("\e[H");

			INFO_OUT("Time: %u, min: %hu (%d, %d), max: %hu (%d, %d)\e[K\n",
					timestamp,
					min, PX_TO_X(min_pix), PX_TO_Y(min_pix),
					max, PX_TO_X(max_pix), PX_TO_Y(max_pix));

			INFO_OUT("Out of range: %d%% mean: %f (%f), median: %d (%f)\e[K\n",
					oor_count * 100 / FREENECT_FRAME_PIX,
					avg, data->depth_lut[(int)avg],
					median, data->depth_lut[median]);

			for(i = 0; i < SM_HIST_SIZE; i++) {
				printf("%*.4f: ", 9, data->depth_lut[i * 2048 / SM_HIST_SIZE]);
				repeat_char(i == median * SM_HIST_SIZE / 2048 ? '*' : '-',
						small_histogram[i] * 96 / FREENECT_FRAME_PIX);
				printf("\e[K\n");
			}
			printf("%*s: ", 9, "Out");
			repeat_char('-', oor_count * 96 / FREENECT_FRAME_PIX);
			printf("\e[K\n");
			
			break;

		case MEDIAN:
			printf("%d\n", median);
			break;

		case MEDIAN_SCALED:
			printf("%f\n", data->depth_lut[median]);
			break;

		case AVERAGE:
			printf("%d\n", (int)avg);
			break;

		case AVERAGE_SCALED:
			printf("%f\n", data->depth_lut[(int)avg]);
			break;
	}

	printf("\e[K");
	fflush(stdout);

	// Make LED red if more than 35% of the image is out of range (can't
	// set LED in callback for some reason)
	data->out_of_range = oor_count > FREENECT_FRAME_PIX * 35 / 100;
}

// Is there a less-global way of passing data to/from a signal handler?
static struct kinstats_info *sigdata;

void intr(int signum)
{
	INFO_OUT("Exiting due to signal %d (%s)\n", signum, strsignal(signum));
	sigdata->done = 1;

	signal(signum, exit);
}

// http://groups.google.com/group/openkinect/browse_thread/thread/31351846fd33c78/e98a94ac605b9f21#e98a94ac605b9f21
void init_lut(float depth_lut[])
{
	int i;

	for(i = 0; i < 2048; i++) {
		depth_lut[i] = 0.1236 * tanf(i / 2842.5 + 1.1863);
	}
}

int main(int argc, char *argv[])
{
	struct kinstats_info data;
	freenect_context *kn;
	freenect_device *kn_dev;
	int opt;

	memset(&data, 0, sizeof(struct kinstats_info));
	sigdata = &data;

	while((opt = getopt(argc, argv, "mMaAv")) != -1) {
		switch(opt) {
			case 'm':
				// Median
				data.disp_mode = MEDIAN;
				break;
			case 'M':
				// Scaled median
				data.disp_mode = MEDIAN_SCALED;
				break;
			case 'a':
				// Mean
				data.disp_mode = AVERAGE;
				break;
			case 'A':
				// Scaled mean
				data.disp_mode = AVERAGE_SCALED;
				break;
			case 'v':
				data.disp_mode = VERBOSE;
				break;
			default:
				fprintf(stderr, "Usage: %s -[mMaAv]\n", argv[0]);
				fprintf(stderr, "Use one of:\n");
				fprintf(stderr, "\tm - median\n");
				fprintf(stderr, "\tM - scaled median\n");
				fprintf(stderr, "\ta - mean\n");
				fprintf(stderr, "\tA - scaled mean\n");
				fprintf(stderr, "\tv - verbose (default)\n");
				return -1;
		}
	}

	if(signal(SIGINT, intr) == SIG_ERR ||
			signal(SIGTERM, intr) == SIG_ERR) {
		ERROR_OUT("Error setting signal handlers\n");
		return -1;
	}

	init_lut(data.depth_lut);

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

	freenect_set_user(kn_dev, &data);
	freenect_set_tilt_degs(kn_dev, -5);
	freenect_set_led(kn_dev, LED_GREEN);
	freenect_set_depth_callback(kn_dev, depth);
	freenect_set_depth_format(kn_dev, FREENECT_DEPTH_11BIT);

	freenect_start_depth(kn_dev);

	printf("\e[H\e[2J");

	int last_oor = data.out_of_range;
	while(!data.done && freenect_process_events(kn) >= 0) {
		if(last_oor != data.out_of_range) {
			freenect_set_led(kn_dev, data.out_of_range ?
					LED_BLINK_RED_YELLOW : LED_GREEN);
			last_oor = data.out_of_range;
		}
	}

	freenect_stop_depth(kn_dev);
	freenect_set_led(kn_dev, LED_OFF);
	freenect_close_device(kn_dev);
	freenect_shutdown(kn);

	return 0;
}

