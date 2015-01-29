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

using IntPoint = eecs467::Point<int>;
using DoublePoint = eecs467::Point<double>;

eecs467::OccupancyGrid grid;

struct Cell_state{
    lcm::LCM occupancy_grid_lcm;
    lcm::LCM pose_lcm;
    lcm_t *rplidar_grid_lcm;
};

void init_main_handlers() {
}
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

    cout << x0 << "," << x1 << "," << y0  << "," <<y1 << endl;

    map<IntPoint, bool> visited;
    for (; n > 0;  n -= 0.01)
    {
        // visit(x, y);

        eecs467::Point<double> p(x, y);
        eecs467::Point<int> cell = global_position_to_grid_cell(p,grid);
        //cout << "cell.x: " << cell.x << "cell.y: " << cell.y << "(" << x << "," << y << ")" << endl;
        if (visited.find(cell) == visited.end()) {
            if (grid(cell.x,cell.y) > -128) {       
                grid(cell.x, cell.y)--;
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
        //cout << "(" << x << "," << y << ")";
    }
    cout << "done looping" << endl;

    //cout <<endl;
    eecs467::Point<double> p(x1, y1);

    eecs467::Point<int> cell = global_position_to_grid_cell(p,grid);
    // cout << "cell.x: " << cell.x << "cell.y: " << cell.y << endl;
    grid(cell.x, cell.y) = grid(cell.x, cell.y) <= 127 - 3 ? grid(cell.x, cell.y) + 3 : 127;
}

void rplidar_grid_handler(const lcm_recv_buf_t *rbuf, const char *channel, const maebot_laser_scan_t *scan, void *user)
{
    printf("Handling rplidar\n");

    //ADD_OBJECT(vxo_line, (vxo_mesh_style(vx_green)));
    int i, npoints;
    float single_line[6]; // x1, y1, z1, x2, y2, z2
    const float* colors[4] = {vx_blue, vx_purple, vx_orange, vx_yellow};

    npoints = 2;

    char rp_buffer[32];
    sprintf(rp_buffer, "rp%d", 0);

    vx_buffer_t *mybuf = vx_world_get_buffer(vx_state.world, rp_buffer);
    //printf("\t%f\t%f\n", matd_get(state.bot, 0, 0), matd_get(state.bot, 1, 0));

    cout << "drawing boundary" << endl;
    for(i = 0; i < scan->num_ranges; ++i){
        // currently centered around origin, will need to be centered around maebot position
        if(scan->intensities[i] <= 0)
            continue;

        float elapsed_time;
        if (scan->times[i] > odo_state.last_updated)
            elapsed_time = scan->times[i] - odo_state.last_updated;
        else
            elapsed_time = -1.0 * (odo_state.last_updated - scan->times[i]);
        elapsed_time /= 1000000.0;


        float x, y;
        x = (scan->ranges[i]) * cosf(scan->thetas[i]);
        y = (scan->ranges[i]) * sinf(scan->thetas[i]);
        rotate_matrix_z(&x, &y, matd_get(state.bot, 2, 0) + elapsed_time * odo_state.v_theta * 0.4);

        fprintf(stderr, "Corrections: x = %f\t y = %f\t delta = %f\n", elapsed_time * odo_state.v_x, elapsed_time * odo_state.v_y, elapsed_time * odo_state.v_theta);

        cerr << "Elapsed time: " << elapsed_time << endl;
        single_line[0] = /*maebot starting x*/ (matd_get(state.bot, 0, 0)) + elapsed_time * odo_state.v_x;
        single_line[1] = /*maebot starting y*/ (matd_get(state.bot, 1, 0)) + elapsed_time * odo_state.v_y;
        single_line[2] = /*maebot starting z*/ 0.0;
        single_line[3] = single_line[0] + x;
        single_line[4] = single_line[1] - y;
        single_line[5] = 0.0;
        
        raytrace(single_line[0], single_line[1], single_line[3], single_line[4]);
        // eecs467::Point<double> p(single_line[3],single_line[4]);

        // eecs467::Point<int> cell = global_position_to_grid_cell(p,grid);
        // // cout << "cell.x: " << cell.x << "cell.y: " << cell.y << endl;
        // if (grid(cell.x,cell.y) < 117) {
        //     grid(cell.x, cell.y) += 10;
        // }
        

        // vx_resc_t *verts = vx_resc_copyf(single_line, npoints*3);
        // vx_object_t *line = vxo_lines(verts, npoints, GL_LINES, 
            // vxo_points_style(colors[state.rp_counter % 4], 2.0f));
        // vx_buffer_add_back(mybuf, line);
    }
    // vx_buffer_swap(mybuf);
    cout << "finish drawing boundary" << endl;
}
void* lcm_rplidar_grid_handler(void *args) {
    Cell_state *lcm_state = (Cell_state*) args;
    //int update_hz = 30;

    while(1){
        lcm_handle(lcm_state->rplidar_grid_lcm);
    }
    return NULL;
}


void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_occupancy_grid_t* msg, void* state) {

}
void* lcm_pose_handler(void *args)
{
    Cell_state *lcm_state = (Cell_state*) args;
    //int update_hz = 30;

    while(1){
        lcm_state->pose_lcm.handle();

    }
    return NULL;
}

void* grid_broadcaster_generator(void* args) {
    Cell_state* cell_state = (Cell_state*) args;
    while(1) {
        maebot_occupancy_grid_t new_grid_msg = grid.toLCM();
        cell_state->occupancy_grid_lcm.publish("OCCUPANCY_GRID", &new_grid_msg);
        usleep(1000000);
    }
    return NULL;
}


int main(int argc, char** argv) {
    init_main_handlers();

    Cell_state cell_state;

    cell_state.rplidar_grid_lcm = lcm_create (NULL);
    if(!cell_state.rplidar_grid_lcm)
        return 1;

    maebot_laser_scan_t_subscribe (cell_state.rplidar_grid_lcm,
        "MAEBOT_LASER_SCAN",
        rplidar_grid_handler,
        NULL);

    cell_state.pose_lcm.subscribeFunction("MAEBOT_POSE", pose_handler, (void*) NULL);

    grid = eecs467::OccupancyGrid(p1_grid_width_c, p1_grid_height_c, p1_cell_sides_width_c);
    // for (int i = 0; i < 10; ++i) {
    //     for (int j = 0; j < 10; ++j) {
    //         grid(j,i) = 12*i;
    //     }        
    // }

    cout << "###########################" << endl;
    /*
    for (int i = 0; i < p1_grid_height_c/p1_cell_sides_width_c; ++i) {
        for (int j = 0; j < p1_grid_width_c/p1_cell_sides_width_c; ++j) {
            cout << (int) grid(j,i) << ",";
        }state.motor_lcm = lcm_create (NULL);
    if(!state.motor_lcm)
        return 1;
        cout << endl;
    }
    */

    pthread_t grid_broadcaster_thread;
    pthread_t rplidar_grid_thread;
    pthread_create(&grid_broadcaster_thread, NULL, grid_broadcaster_generator, (void*)(&cell_state));
    pthread_create(&rplidar_grid_thread, NULL, lcm_rplidar_grid_handler, (void*)(&cell_state));

    Maebot_View maebot_world;
    maebot_world.start(argc, argv);
}

