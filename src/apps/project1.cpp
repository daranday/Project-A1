#include "maebot_view.h"

#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>

#include "../mapping/occupancy_grid_utils.hpp"

using namespace std;

const float grid_width_c  = 10;
const float grid_height_c = 10;
const float cell_sides_width_c = 0.05;

OccupancyGrid grid;

void main_motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_motor_feedback_t *msg, void *user) {	
	motor_feedback_handler(rbuf, channel, msg, user);
}

void main_rplidar_feedback_handler(const lcm_recv_buf_t *rbuf, const char *channel, const maebot_laser_scan_t *scan, void *user) {
	rplidar_feedback_handler(rbuf, channel, scan, user);
}

void init_main_handlers() {
	state.motor_lcm = lcm_create (NULL);
	if(!state.motor_lcm)
		return 1;
	
	state.lidar_lcm = lcm_create(NULL);
	if(!state.lidar_lcm)
		return 1;

	maebot_motor_feedback_t_subscribe (state.motor_lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		main_motor_feedback_handler,
		NULL);
	maebot_laser_scan_t_subscribe (state.lidar_lcm,
		"MAEBOT_LASER_SCAN",
		main_rplidar_feedback_handler,
		NULL);

	pthread_t lcm_lidar_thread, lcm_motor_thread;
	pthread_create(&lcm_motor_thread, NULL, lcm_motor_handler, (void*)(&state));
	pthread_create(&lcm_lidar_thread, NULL, lcm_lidar_handler, (void*)(&state));
}

int main(int argc, char** argv) {
	init_main_handlers();
	grid = OccupancyGrid(grid_width_c, grid_height_c, cell_sides_width_c);



	Maebot_View maebot_world;
	maebot_world.start(argc, argv);
}

