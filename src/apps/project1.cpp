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
    cout << "hellolasergrid" << endl;
    for(int i = 0; i < scan->num_ranges; ++i){
        if(scan->intensities[i] <= 0)
            continue;
        
        float single_line[4], elapsed_time; 

        // Calculated Elapsed Time.

        int64_t last_updated_time = pose_state.last_updated;// odo_state.last_updated

        if (scan->times[i] > last_updated_time)
            elapsed_time = scan->times[i] - last_updated_time;
        else
            elapsed_time = -1.0 * (last_updated_time - scan->times[i]);
        elapsed_time /= 1000000.0;

        float x, y;
        x = (scan->ranges[i]) * cosf(scan->thetas[i]);
        y = (scan->ranges[i]) * sinf(scan->thetas[i]);
        rotate_matrix_z(&x, &y, pose_state.theta + elapsed_time * pose_state.v_theta);

        // fprintf(stderr, "Corrections: x = %f\t y = %f\t delta = %f\n", elapsed_time * pose_state.v_x, elapsed_time * pose_state.v_y, elapsed_time * pose_state.v_theta);

        // cerr << "Elapsed time: " << elapsed_time << endl;
        single_line[0] = pose_state.x + elapsed_time * pose_state.v_x;
        single_line[1] = pose_state.y + elapsed_time * pose_state.v_y;
        single_line[2] = single_line[0] + x;
        single_line[3] = single_line[1] - y;

        printf("%.4f\t%.4f\t%.4f\t%.4f\n", pose_state.v_x, pose_state.v_y, pose_state.v_theta, elapsed_time);
        printf("%.4f\t%.4f\t%.4f\t%.4f\n", single_line[0], single_line[1], single_line[2], single_line[3]);
        
        raytrace(single_line[0], single_line[1], single_line[2], single_line[3]);
    }
    cout << "byelasergrid" << endl;
}

void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_pose_t* msg, void* user) {
    cout << "hellopose" << endl;
    float time_elapsed;
    int64_t last_updated_time = pose_state.last_updated;// odo_state.last_updated

    if (msg->utime > last_updated_time)
        time_elapsed = msg->utime - last_updated_time;
    else
        time_elapsed = -1.0 * (last_updated_time - msg->utime);

    time_elapsed /= 1000000.0;
    float speed = sqrt((msg->x - pose_state.x) * (msg->x - pose_state.x) + (msg->y - pose_state.y) * (msg->y - pose_state.y)) / time_elapsed;
    pose_state.v_x = speed * cosf(msg->theta);
    pose_state.v_y = speed * sinf(msg->theta);
    pose_state.v_theta = (msg->theta - pose_state.theta) / time_elapsed;

    pose_state.x = msg->x;
    pose_state.y = msg->y;
    pose_state.theta = msg->theta;
    pose_state.last_updated = msg->utime;

    // Update Vx World
    char odo_buffer[32];
    float current_position[3] = {state.scale * (float)pose_state.x, state.scale * (float)pose_state.y, 0.0};
    sprintf(odo_buffer, "odo%d", state.odo_counter++);

    vx_resc_t *one_point = vx_resc_copyf(current_position,3);
    vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, odo_buffer);
    vx_object_t *trace = vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)); 
    // vxo_chain(vxo_mat_translate3(state.bot.x, state.bot.y, 0.0),
    // vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
    vx_buffer_add_back(buf, trace);
    vx_buffer_swap(buf);

    // printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4fs\n", pose_state.x, pose_state.y, pose_state.theta, pose_state.v_x, pose_state.v_y, pose_state.v_theta, time_elapsed);
    cout << "byepose" << endl;
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

