#include "maebot_view.h"

#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>

#include "../mapping/occupancy_grid_utils.hpp"

using namespace std;

const float p1_grid_width_c  = 1;
const float p1_grid_height_c = 1;
const float p1_cell_sides_width_c = 0.05;

eecs467::OccupancyGrid grid;

void init_main_handlers() {
}


void* grid_broadcaster_generator(void* args) {
    while(1) {
        maebot_occupancy_grid_t new_grid_msg = grid.toLCM();
        state.lcm.publish("OCCUPANCY_GRID", &new_grid_msg);
        usleep(5000000);
    }
    return NULL;
}


int main(int argc, char** argv) {
	init_main_handlers();
	grid = eecs467::OccupancyGrid(p1_grid_width_c, p1_grid_height_c, p1_cell_sides_width_c);
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            grid(j,i) = -128;
        }        
    }

    cout << "###########################" << endl;
    
    for (int i = 0; i < p1_grid_height_c/p1_cell_sides_width_c; ++i) {
        for (int j = 0; j < p1_grid_width_c/p1_cell_sides_width_c; ++j) {
            cout << (int) grid(j,i) << ",";
        }
        cout << endl;
    }

    pthread_t grid_broadcaster_thread;
    pthread_create(&grid_broadcaster_thread, NULL, grid_broadcaster_generator, (void*)(&state));

	Maebot_View maebot_world;
	maebot_world.start(argc, argv);
}

