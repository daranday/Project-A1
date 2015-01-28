#ifndef __MAEBOT_VIEW__
#define __MAEBOT_VIEW__

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>

#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"
#include "../math/matd.h"
#include "../math/fasttrig.h"


#include <gtk/gtk.h>
#include "vx/vxo_drawables.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_global.h"
#include "vx/vx_layer.h"
#include "vx/vx_world.h"
#include "vx/vx_colors.h"

#include "vx/vx_camera_mgr.h"

// image
#include "imagesource/image_u8.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// map
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
//#include "lcmtypes/maebot_occupancy_grid_t.h"

#define WHEEL_BASE 0.08f
#define WHEEL_DIAMETER 0.032f
#define DISTANCE_TICK 0.0002094f
#define GYRO_2_RADS 0.0001343f
#define ACCEL_2_MET	0.000598f

#define  ADD_OBJECT(s, call)					    \
{								    \
	obj_data_t data = {.obj = s call, .name = #s};		    \
	zarray_add(vx_state.obj_data, &data);				    \
}


const float grid_width_c  = 10;
const float grid_height_c = 10;
const float cell_sides_width_c = 0.05;

class Maebot_View {
public:
	int start (int argc, char *argv[]);
};

struct vx_state_t{
	zarray_t *obj_data;
	vx_world_t *world;
};

extern vx_state_t vx_state;

struct State{
    lcm::LCM occupancy_grid_lcm;
	lcm_t *motor_lcm;
	lcm_t *lidar_lcm;
	lcm_t *imu_lcm;
    //lcm_t *occupancy_grid_lcm;
	
	matd_t* bot; // 3x1 state [x][y][theta]
	//char buffer[32];
	int odo_counter;
	int rp_counter;
	int imu_counter;
};

extern State state;

//previous odometry reading
struct Odo_state{
	int32_t left;
	int32_t right;
	int8_t init;

	int32_t delta_left;
	int32_t delta_right;
	float delta_s;
	float delta_s_l;
	float delta_s_r;
	float delta_x;
	float delta_y;
	float delta_theta;
};

extern Odo_state odo_state;

struct IMU_State {
  matd_t *bot; // 3x2 state [x, Vx][y, Vy][theta, Vtheta]
  int64_t prev_time;
  
  float v_x_i;
  float v_y_i;
  float v_theta_i;
  float delta_x_local;
  float delta_y_local;
  float delta_x_global;
  float delta_y_global;
  float delta_theta;
  float delta_time;
  float delta_time_squared;
  float a_x;
  float a_y;
  float v_theta;
};

extern IMU_State imu_state;

struct Occupancy_Grid_State {
    eecs467::OccupancyGrid* grid; 
    Occupancy_Grid_State() : grid(new eecs467::OccupancyGrid(grid_width_c, grid_height_c, cell_sides_width_c)){}
    ~Occupancy_Grid_State() { delete grid;}
};

extern Occupancy_Grid_State occupancy_grid_state;

struct obj_data_t{
	vx_object_t *obj;
	char *name;
};


void draw(vx_world_t * world, zarray_t * obj_data);
void motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_motor_feedback_t *msg, void *user);
void rotate_matrix_z(float* x, float* y, float theta);
void rplidar_feedback_handler(const lcm_recv_buf_t *rbuf, const char *channel, const maebot_laser_scan_t *scan, void *user);
void sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_sensor_data_t *msg, void *user);
void occupancy_grid_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_occupancy_grid_t* msg, void* state);
void* lcm_lidar_handler(void *args);
void* lcm_motor_handler(void *args);
void* lcm_imu_handler(void *args);
void* lcm_occupancy_grid_handler(void *args);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);


#endif