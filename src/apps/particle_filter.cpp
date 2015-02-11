// #include "maebot_view.h"
#include "particle_filter.h"
#include "../math/angle_functions.hpp"
#include "../math/gsl_util_rand.h"

#include <vector>

using namespace std;

// const int NUM_PARTICLE = 1000;
// const int GREY_THRESH = 85;
// const int WHITE_THRESH = 170;
//std::default_random_engine generator; // Returns unsigned int

// struct Particle_t {
//     float x;
//     float y;
//     float theta;
//     float weight;
//     Particle_t() :x(0), y(0), theta(0), weight(0) {}
//     Particle_t(float x_in, float y_in, float t_in, float w_in) : 
//                                         x(x_in), y(y_in), theta(t_in), weight(w_in) {}
// };

vector <Particle_t> particles(NUM_PARTICLE);

/*
void init_particles() {
    for (int i = 0; i < particles.size(); ++i) {
        particles[i] = Particle_t();
    }
}
*/


void update_particle_weight(Particle_t& particle, DoublePoint& laser_end_position) {
    //DoublePoint laser_end_position = DoublePoint(particle.x + delta_r.x, particle.y + delta_r.y);
    IntPoint laser_end_cell = eecs467::global_position_to_grid_cell(laser_end_position, state.grid);

    if (state.grid.isCellInGrid(laser_end_cell.x, laser_end_cell.y)) {
        if (state.grid(laser_end_cell.x, laser_end_cell.y) < 63) {
            particle.weight -= -12;
        } else if (state.grid(laser_end_cell.x, laser_end_cell.y) >= 64) {
            particle.weight -= -4;
        } else {
            particle.weight -= -8;
        }
    }
    else {
        particle.weight -= -12;
    }
}

void sensor_model_updater(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user)
{
    cout << "doing sensor model" << endl;

    float max_weight = FLT_MIN; // minimum float nubmer

    int counts = 0;
    vx_buffer_t * mybuf = vx_world_get_buffer(vx_state.world, "Yellow Laser");
    vx_buffer_t * mybuf2 = vx_world_get_buffer(vx_state.world, "current particle");

    for (int j = 0, len = particles.size(); j < len; ++j) {
        for(int i = 0; i < scan->num_ranges; ++i){
            if(scan->intensities[i] <= 0)
                continue;
            int64_t last_updated_time = odo_state.last_updated;// odo_state.last_updated
            float single_line[4], plot_line[4], elapsed_time; 
            float x, y;
            
            elapsed_time = (scan->times[i] - last_updated_time) / 1000000.0;

            x = (scan->ranges[i]) * cosf(scan->thetas[i]);
            y = (scan->ranges[i]) * sinf(scan->thetas[i]);

            // use odostate velocity

            // ######## use odostate theta don't consider the particle's theta ##########
            //rotate_matrix_z(&x, &y, eecs467::angle_sum(odo_state.theta, elapsed_time * odo_state.v_theta);
            rotate_matrix_z(&x, &y, eecs467::angle_sum(particles[j].theta, elapsed_time * odo_state.v_theta));

            // ######## use odostate position don't consider the particle's position ##########
            //single_line[0] = odo_state.x + elapsed_time * odo_state.v_x;
            //single_line[1] = odo_state.y + elapsed_time * odo_state.v_y;

            single_line[0] = particles[j].x + elapsed_time * odo_state.v_x;
            single_line[1] = particles[j].y + elapsed_time * odo_state.v_y;
            single_line[2] = single_line[0] + x;
            single_line[3] = single_line[1] - y;

            
            plot_line[0] = state.scale * (particles[j].x + elapsed_time * odo_state.v_x);
            plot_line[1] = state.scale * (particles[j].y + elapsed_time * odo_state.v_y);
            plot_line[2] = plot_line[0] + state.scale * x;
            plot_line[3] = plot_line[1] - state.scale * y;

            if (fabs(odo_state.v_theta) > 10 )
                return;

            // update weight for particles
            
            vx_resc_t *verts = vx_resc_copyf(plot_line, 2 * 2);
            vx_object_t *line = vxo_lines(verts, 2, GL_LINES, vxo_points_style(counts < 290/3 ? vx_green : counts < 290*2/3 ? vx_blue : vx_yellow, 2.0f));
            vx_buffer_add_back(mybuf, line);
            counts++;
            
            //DoublePoint delta_r(single_line[2] - single_line[0], single_line[3] - single_line[1]);
            DoublePoint laser_end_position(single_line[2], single_line[3]);
            
            update_particle_weight(particles[j], laser_end_position);

        }
        if (particles[j].weight > max_weight) {
            max_weight = particles[j].weight;
        }

        vx_buffer_swap(mybuf);

        
        float current_position[3] = {state.scale * (float)particles[j].x, state.scale * (float)particles[j].y, 0.05};
        vx_resc_t *one_point = vx_resc_copyf(current_position,3);
        vx_object_t *trace = vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)); 
        // vxo_chain(vxo_mat_translate3(state.bot.x, state.bot.y, 0.0),
        // vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
        vx_buffer_add_back(mybuf2, trace);
    }
    vx_buffer_swap(mybuf2);

    cout << "before normalize" << endl;
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
    }

    //normalize
    float weight_sum = 0;
    for (int j = 0, len = particles.size(); j < len; ++j) {
        float norm_weight = pow (10, particles[j].weight - max_weight); // assume log base 10

        particles[j].weight = norm_weight;
        weight_sum += norm_weight;
    }
    // make sure all sum up to 1
    for (int j = 0, len = particles.size(); j < len; ++j) {
        particles[j].weight /= weight_sum;
    }

    cout << "after normalize" << endl;
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
    }

}

void action_model_updater (const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_motor_feedback_t *msg, void *user)
{
    cout << "doing action model" << endl;

    float prev_x = odo_state.x;
    float prev_y = odo_state.y;
    float prev_theta = odo_state.theta;

    int delta_left = msg->encoder_left_ticks - odo_state.left;
    int delta_right = msg->encoder_right_ticks - odo_state.right;
    
    float delta_s_l = (DISTANCE_TICK * delta_left);
    float delta_s_r = (DISTANCE_TICK * delta_right);

    float delta_s =(delta_s_l + delta_s_r)/2.0;
    float delta_theta = (delta_s_r - delta_s_l)/WHEEL_BASE + odo_state.theta;
    
    if(delta_s < 0)
        delta_s = delta_s*(-1.0);
    
    float delta_x = delta_s*fcos(delta_theta) + odo_state.x;
    float delta_y = delta_s*fsin(delta_theta) + odo_state.y;

    odo_state.x = delta_x;
    odo_state.y = delta_y;
    odo_state.theta = delta_theta;

    float left_speed = msg->motor_left_actual_speed;
    float right_speed = msg->motor_right_actual_speed;
    float speed = (left_speed + right_speed) / 2.0;

    odo_state.v_x = speed * fcos(delta_theta);
    odo_state.v_y = speed * fsin(delta_theta);
    odo_state.v_theta = (right_speed - left_speed) / WHEEL_BASE;
    
    odo_state.left = msg->encoder_left_ticks;
    odo_state.right = msg->encoder_right_ticks;


    // // Update Vx World
    // char odo_buffer[32];
    // float current_position[3] = {state.scale * (float)odo_state.x, state.scale * (float)odo_state.y, 0.0};
    // sprintf(odo_buffer, "odo%d", state.odo_counter++);

    // vx_resc_t *one_point = vx_resc_copyf(current_position,3);
    // vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, odo_buffer);
    // vx_object_t *trace = vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)); 
    // // vxo_chain(vxo_mat_translate3(odo_state.x, odo_state.y, 0.0),
    // // vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
    // vx_buffer_add_back(buf, trace);
    // vx_buffer_swap(buf);


    delta_x = odo_state.x - prev_x;
    delta_y = odo_state.y - prev_y;
    float alpha = eecs467::angle_diff(atan2(delta_y, delta_x), prev_theta);
    float s = sqrt((delta_x)*(delta_x) + (delta_y)*(delta_y));
    float phi = eecs467::angle_diff(odo_state.theta, prev_theta);
    // action_state.last_updated = msg->utime;


    // sample
    vector<int> sampled_particles(NUM_PARTICLE);
    double step = 1.0/NUM_PARTICLE;
    double weight_sum = particles[0].weight;
    double step_sum = step/2;
    int index = 0;
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        //cout << "before weight_sum " << weight_sum << " step_sum " << step_sum << " index " << index << " i " << i << endl;
    
        while (weight_sum <= step_sum) {
            //cout << " particles[index].weight " << particles[index].weight << endl;

            weight_sum += particles[++index].weight;
        }
        sampled_particles[i] = index;
        step_sum += step;

        //cout << "weight_sum " << weight_sum << " step_sum " << step_sum << " index " << index << " i " << i << endl;
    
    }

/*
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
        cout << "sample : " << sampled_particles[i] << endl;
    }
*/

    // update
    gsl_rng * rng = gslu_rand_rng_alloc();

    const float k1 = 1.0;
    const float k2 = 0.4;
    double e1 = 0; 
    double e2 = 0;
    double e3 = 0;
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        e1 = gslu_rand_gaussian(rng, 0, k1*alpha);
        e2 = gslu_rand_gaussian(rng, 0, k2*s);
        e3 = gslu_rand_gaussian(rng, 0, k1*(phi-alpha));

        Particle_t& sampled_particle = particles[sampled_particles[i]];
        float theta_e1 = eecs467::angle_sum(sampled_particle.theta, e1);

        particles[i].x = sampled_particle.x + (s + e2)*cos(eecs467::angle_sum(theta_e1, alpha));
        particles[i].y = sampled_particle.y + (s + e2)*sin(eecs467::angle_sum(theta_e1, alpha));
        particles[i].theta = eecs467::angle_sum(eecs467::angle_sum(theta_e1, phi), e3);
    }
    delete rng;

    // float current_position[3] = {state.scale * (float)pose_state.x, state.scale * (float)pose_state.y, 0.05};
    vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, "particles");

    for (int i = 0; i < particles.size(); i++) {
        float current_position[3] = {state.scale * (float)particles[i].x, state.scale * (float)particles[i].y, 0.05};
        
        //cout << current_position[0]  << "," << current_position[1]  <<"," <<current_position[2] << "|";

        vx_resc_t *one_point = vx_resc_copyf(current_position,3);
        vx_object_t *trace = vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f));
        vx_buffer_add_back(buf, trace);

    }
    //cout << endl;
    // vxo_chain(vxo_mat_translate3(state.bot.x, state.bot.y, 0.0),
    // vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
    // vx_buffer_add_back(buf, trace);
    vx_buffer_swap(buf);
}

/*
Init all to zero

loop through each particle
    sample particle = sample(old particle)
    new particle = action model(sample particle)
    loop through laser scan
        end coord = particle pose + laser scan + correction
        if end coord == wall
            weight += sth
        else if end coord == grey
            weight += sth
        else 
            weight += sth
    
    total weight += weight
    add new particle to new vector
    
loop through each new particle
    weight/total weight
    */