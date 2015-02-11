#ifndef __PARTICLE_FILTER__
#define __PARTICLE_FILTER__

#include "maebot_view.h"

const int NUM_PARTICLE = 5; // 1000
const int GREY_THRESH = 85;
const int WHITE_THRESH = 170;

typedef eecs467::Point<int> IntPoint;
typedef eecs467::Point<double> DoublePoint;

struct Particle_t {
    float x;
    float y;
    float theta;
    float weight;
    Particle_t() :x(0), y(0), theta(0), weight(1.0/NUM_PARTICLE) {}
    Particle_t(float x_in, float y_in, float t_in, float w_in) : 
                                        x(x_in), y(y_in), theta(t_in), weight(w_in) {}
};


void update_particle_weight(Particle_t& particle, DoublePoint& delta_r);
void sensor_model_updater(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user);
void action_model_updater (const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_motor_feedback_t *msg, void *user);


#endif