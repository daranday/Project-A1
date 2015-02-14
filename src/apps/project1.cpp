#include "maebot_view.h"
#include "particle_filter.h"

#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include "../math/point.hpp"
#include <random>

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
        eecs467::Point<int> cell = global_position_to_grid_cell(p,state.grid);
        //cout << "cell.x: " << cell.x << "cell.y: " << cell.y << "(" << x << "," << y << ")" << endl;
        if (visited.find(cell) == visited.end()) {
            if (state.grid(cell.x,cell.y) > -128) {       
                state.grid(cell.x, cell.y)--;
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

    eecs467::Point<int> cell = global_position_to_grid_cell(p,state.grid);
    state.grid(cell.x, cell.y) = state.grid(cell.x, cell.y) <= 127 - 3 ? state.grid(cell.x, cell.y) + 3 : 127;
}

void laser_update_grid_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user)
{
    // vx_buffer_t *mybuf = vx_world_get_buffer(vx_state.world, "Yellow Laser");
    // cout << "hellolasergrid" << endl;
    int counts = 0;
    for(int i = 0; i < scan->num_ranges; ++i){
        if(scan->intensities[i] <= 0)
            continue;
        
        float single_line[4], elapsed_time, plot_line[4];
        int64_t last_updated_time = odo_state.last_updated;// odo_state.last_updated

        // Calculated Elapsed Time.
        elapsed_time = (scan->times[i] - last_updated_time ) / 1000000.;

        float x, y;
        x = (scan->ranges[i]) * cosf(scan->thetas[i]);
        y = (scan->ranges[i]) * sinf(scan->thetas[i]);
        rotate_matrix_z(&x, &y, odo_state.theta + 0.4 * elapsed_time * odo_state.v_theta);

        plot_line[0] = state.scale * (odo_state.x + elapsed_time * odo_state.v_x);
        plot_line[1] = state.scale * (odo_state.y + elapsed_time * odo_state.v_y);
        plot_line[2] = plot_line[0] + state.scale * x;
        plot_line[3] = plot_line[1] - state.scale * y;


        single_line[0] = odo_state.x + elapsed_time * odo_state.v_x;
        single_line[1] = odo_state.y + elapsed_time * odo_state.v_y;
        single_line[2] = single_line[0] + x;
        single_line[3] = single_line[1] - y;

        if (fabs(odo_state.v_theta) > 10)
            return;

        
        // vx_resc_t *verts = vx_resc_copyf(plot_line, 2 * 2);
        // vx_object_t *line = vxo_lines(verts, 2, GL_LINES, vxo_points_style(counts < 290/3 ? vx_green : counts < 290*2/3 ? vx_blue : vx_yellow, 2.0f));
        // vx_buffer_add_back(mybuf, line);
        counts++;

        raytrace(single_line[0], single_line[1], single_line[2], single_line[3]);
    }
    cout << "-----------" << counts << endl;
    // vx_buffer_swap(mybuf);
    // cout << "byelasergrid" << endl;
}

void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_pose_t* msg, void* user) {
    // cout << "hellopose" << endl;
    int64_t last_updated_time = pose_state.last_updated;// odo_state.last_updated
    if (last_updated_time != 0) {
        float   time_elapsed      = (msg->utime - last_updated_time) / 1000000.;

        float speed = sqrt((msg->x - pose_state.x) * (msg->x - pose_state.x) + (msg->y - pose_state.y) * (msg->y - pose_state.y)) / time_elapsed;
        pose_state.v_x = speed * cosf(msg->theta);
        pose_state.v_y = speed * sinf(msg->theta);
        pose_state.v_theta = eecs467::angle_diff(msg->theta, pose_state.theta) / time_elapsed;
    }

    pose_state.x = msg->x;
    pose_state.y = msg->y;
    pose_state.theta = msg->theta;
    pose_state.last_updated = msg->utime;

    // Update Vx World
    char odo_buffer[32];
    float current_position[3] = {state.scale * (float)pose_state.x, state.scale * (float)pose_state.y, 0.05};
    sprintf(odo_buffer, "odo%d", state.odo_counter++);

    vx_resc_t *one_point = vx_resc_copyf(current_position,3);
    vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, odo_buffer);
    vx_object_t *trace = vxo_points(one_point, 1, vxo_points_style(vx_green, 2.0f)); 
    // vxo_chain(vxo_mat_translate3(state.bot.x, state.bot.y, 0.0),
    // vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
    vx_buffer_add_back(buf, trace);
    vx_buffer_swap(buf);

    // printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4fs\n", pose_state.x, pose_state.y, pose_state.theta, pose_state.v_x, pose_state.v_y, pose_state.v_theta, time_elapsed);
    // cout << "byepose" << endl;
}

void* grid_broadcast_generator(void* args) {
    double fps = 30;
    while(1) {
        maebot_occupancy_grid_t new_grid_msg = state.grid.toLCM();
        state.lcm.publish("OCCUPANCY_GRID", &new_grid_msg);
        usleep(1000000./fps);
    }
    return NULL;
}

void init_main_handlers() {
    state.lcm.subscribeFunction("MAEBOT_POSE", pose_handler, (void*) NULL);
    state.lcm.subscribeFunction("OCCUPANCY_GRID", occupancy_grid_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_LASER_SCAN", laser_update_grid_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_LASER_SCAN", sensor_model_updater, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_MOTOR_FEEDBACK", motor_feedback_handler, (void*) NULL);
    // state.lcm.subscribeFunction("MAEBOT_LASER_SCAN", rplidar_feedback_handler, (void*) NULL);
    state.lcm.subscribeFunction("MAEBOT_MOTOR_FEEDBACK", action_model_updater, (void*) NULL);
    // state.lcm.subscribeFunction("MAEBOT_SENSOR_DATA", sensor_data_handler, (void*) NULL);
}

void read_map() {
    float map_width, map_height, cell_side;
    int grid_width, grid_height, cell_odds;
    ifstream fin("maps/figure8_map.txt");

    fin >> map_width >> map_height >> cell_side;
    fin >> grid_width >> grid_height;

    //cout << map_width << "," << map_height << "," << cell_side << endl;
    //cout << grid_width << "," << grid_height << endl;

    state.grid = eecs467::OccupancyGrid(map_width, map_height, cell_side);
    for (int i = 0; i < state.grid.heightInCells(); ++i) {
        for (int j = 0; j < state.grid.widthInCells(); ++j) {
            fin >> cell_odds;
            //cout << "(" << cell_odds << "," << j << "," << i << ")";
            state.grid(j, i) = (int8_t)cell_odds;
        }
        //cout << endl;
    }
    fin.close();
}

int main(int argc, char** argv) {
    init_main_handlers();

    pthread_t grid_broadcast_thread;
    pthread_create(&grid_broadcast_thread, NULL, grid_broadcast_generator, (void*)NULL);

    Maebot_View maebot_world;
    maebot_world.start(argc, argv);
}

