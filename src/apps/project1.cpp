#include "maebot_view.h"

#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include "../math/point.hpp"


#include "../mapping/occupancy_grid_utils.hpp"

using namespace std;

const float p1_grid_width_c  = 4;
const float p1_grid_height_c = 4;
const float p1_cell_sides_width_c = 0.05;

typedef eecs467::Point<int> IntPoint;
typedef eecs467::Point<double> DoublePoint;

void raytrace(double x0, double y0, double x1, double y1)
{
    double dx = abs(x1 - x0);
    double dy = abs(y1 - y0);
    double x = x0;
    double y = y0;
    double n = 0.01 + dx + dy;
    double x_inc = (x1 > x0) ? 0.01 : -0.01;
    double y_inc = (y1 > y0) ? 0.01 : -0.01;
    double error = dx - dy;
    dx *= 2;
    dy *= 2;

    // cout << x0 << "," << x1 << "," << y0  << "," <<y1 << endl;

    map<IntPoint, bool> visited;
    for (; n > 0;  n -= 0.01)
    {
        // visit(x, y);

        eecs467::Point<double> p(x, y);
        eecs467::Point<int> cell = global_position_to_grid_cell(p,occupancy_grid_state.grid);
        //cout << "cell.x: " << cell.x << "cell.y: " << cell.y << "(" << x << "," << y << ")" << endl;
        if (visited.find(cell) == visited.end()) {
            if (occupancy_grid_state.grid(cell.x,cell.y) > -128) {       
                occupancy_grid_state.grid(cell.x, cell.y)--;
            }
            visited[cell] = true;
        }
        
        if (error > 0)
        {
            x += x_inc;
            error -= dy;
        }

        else
        {
            y += y_inc;
            error += dx;
        }
    }

    eecs467::Point<double> p(x1, y1);

    eecs467::Point<int> cell = global_position_to_grid_cell(p,occupancy_grid_state.grid);
    occupancy_grid_state.grid(cell.x, cell.y) = occupancy_grid_state.grid(cell.x, cell.y) <= 127 - 3 ? occupancy_grid_state.grid(cell.x, cell.y) + 3 : 127;
}

void laser_update_grid_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user)
{
    for(int i = 0; i < scan->num_ranges; ++i){
        if(scan->intensities[i] <= 0)
            continue;
        
        float single_line[4], elapsed_time; 

        // Calculated Elapsed Time.
        if (scan->times[i] > odo_state.last_updated)
            elapsed_time = scan->times[i] - odo_state.last_updated;
        else
            elapsed_time = -1.0 * (odo_state.last_updated - scan->times[i]);
        elapsed_time /= 1000000.0;


        float x, y;
        x = (scan->ranges[i]) * cosf(scan->thetas[i]);
        y = (scan->ranges[i]) * sinf(scan->thetas[i]);
        rotate_matrix_z(&x, &y, state.bot.theta + elapsed_time * odo_state.v_theta * 0.4);

        // fprintf(stderr, "Corrections: x = %f\t y = %f\t delta = %f\n", elapsed_time * odo_state.v_x, elapsed_time * odo_state.v_y, elapsed_time * odo_state.v_theta);

        // cerr << "Elapsed time: " << elapsed_time << endl;
        single_line[0] = state.bot.x + elapsed_time * odo_state.v_x;
        single_line[1] = state.bot.y + elapsed_time * odo_state.v_y;
        single_line[2] = single_line[0] + x;
        single_line[3] = single_line[1] - y;
        
        raytrace(single_line[0], single_line[1], single_line[2], single_line[3]);
    }
}

void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_pose_t* msg, void* user) {
}

void* grid_broadcast_generator(void* args) {
    while(1) {
        maebot_occupancy_grid_t new_grid_msg = occupancy_grid_state.grid.toLCM();
        state.lcm.publish("OCCUPANCY_GRID", &new_grid_msg);
        usleep(1000000);
    }
    return NULL;
}

void init_main_handlers() {
    state.lcm.subscribeFunction("MAEBOT_MOTOR_FEEDBACK", motor_feedback_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_LASER_SCAN", rplidar_feedback_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_SENSOR_DATA", sensor_data_handler, (void*) NULL);
    state.lcm.subscribeFunction("OCCUPANCY_GRID", occupancy_grid_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_LASER_SCAN", laser_update_grid_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_POSE", pose_handler, (void*) NULL);
}

int main(int argc, char** argv) {
    init_main_handlers();

    pthread_t grid_broadcast_thread;
    pthread_create(&grid_broadcast_thread, NULL, grid_broadcast_generator, (void*)NULL);

    Maebot_View maebot_world;
    maebot_world.start(argc, argv);
}

