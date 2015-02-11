#ifndef __MAEBOT_VIEW__
#define __MAEBOT_VIEW__

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "../math/matd.h"
#include "../math/fasttrig.h"
#include "../math/angle_functions.hpp"

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

struct Pose_t {
  float x;
  float y;
  float theta;
  Pose_t() :x(0), y(0), theta(0) {}
};

struct Pose_state_t : Pose_t {
  float v_x;
  float v_y;
  float v_theta;

  int64_t last_updated;
  Pose_state_t() : Pose_t(), last_updated(0) {}
};

extern Pose_state_t pose_state;

struct State{
  lcm::LCM lcm;

	int odo_counter;
	int rp_counter;
	int imu_counter;

  pthread_mutex_t state_lock;

  float scale;

  eecs467::OccupancyGrid grid;

  State() :odo_counter(0), rp_counter(0), imu_counter(0), scale(8.0), grid(eecs467::OccupancyGrid(grid_width_c, grid_height_c, cell_sides_width_c)){}
};

extern State state;

//previous odometry reading
struct Odo_state : Pose_state_t{
	int32_t left;
	int32_t right;
	int8_t init;

  pthread_mutex_t odo_lock;
};

extern Odo_state odo_state;

struct Action_state {
    float alpha;
    float s;
    float phi;
    float cur_time;

    int64_t last_updated;
};

extern Action_state action_state;






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

  IMU_State() {
    bot = matd_create(3,2);
    prev_time = 0.0;
  }
};

extern IMU_State imu_state;

struct Occupancy_Grid_State {
    ~Occupancy_Grid_State() {}
};

extern Occupancy_Grid_State occupancy_grid_state;
/*
struct Particle_State {
    std::set< > particle;
};

extern Particle_State particle_state;
*/
struct obj_data_t{
	vx_object_t *obj;
	char *name;
};

void rotate_matrix_z(float* x, float* y, float theta);
void rplidar_feedback_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user);
void motor_feedback_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_motor_feedback_t *msg, void *user);
void sensor_data_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_sensor_data_t *msg, void *user);
void occupancy_grid_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_occupancy_grid_t* msg, void* user);
void* lcm_handler(void *args);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);
void draw(vx_world_t * world, zarray_t * obj_data);


#endif