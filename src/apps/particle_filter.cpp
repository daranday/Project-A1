// #include "maebot_view.h"
#include "particle_filter.h"
#include "../math/angle_functions.hpp"
#include "../math/gsl_util_rand.h"

#include <vector>

using namespace std;

vector <Particle_t> particles(NUM_PARTICLE);

void update_particle_weight(Particle_t& particle, DoublePoint& laser_end_position) {
    IntPoint laser_end_cell = eecs467::global_position_to_grid_cell(laser_end_position, state.grid);

    if (state.grid.isCellInGrid(laser_end_cell.x, laser_end_cell.y)) {
        if (state.grid(laser_end_cell.x, laser_end_cell.y) >= GREY_BLACK_THRESH) {
            particle.weight -= 4;
        } else if (state.grid(laser_end_cell.x, laser_end_cell.y) >= WHITE_GREY_THRESH) {
            particle.weight -= 12;
        } else {
            particle.weight -= 8;
        }
    }
    else {
        particle.weight -= 12;
    }
}


void update_slam_state(Particle_t &best_particle, int64_t current_time) {
    // Update velocity
    if (slam_state.last_updated != 0) {
        float time_elapsed = (current_time - slam_state.last_updated) / 1000000.;
        float speed = sqrt((best_particle.x - slam_state.x) * (best_particle.x - slam_state.x) + (best_particle.y - slam_state.y) * (best_particle.y - slam_state.y)) / time_elapsed;

        slam_state.v_x = speed * cosf(best_particle.theta);
        slam_state.v_y = speed * sinf(best_particle.theta);
        slam_state.v_theta = eecs467::angle_diff(best_particle.theta, slam_state.theta) / time_elapsed;
    }
    // Update position
    slam_state.x = best_particle.x;
    slam_state.y = best_particle.y;
    slam_state.theta = best_particle.theta;
    slam_state.last_updated = current_time;
}

void sensor_model_updater(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user)
{
    // cout << "doing sensor model" << endl;
    double max_weight = -std::numeric_limits<double>::max(); // minimum float nubmer
    int best_particle_index;

    vx_buffer_t * laser_buffer = vx_world_get_buffer(vx_state.world, "Sensor Model Laser");
    vx_buffer_t * particles_buffer = vx_world_get_buffer(vx_state.world, "Sensor Model Particles");
    vx_buffer_t * best_particle_buffer = vx_world_get_buffer(vx_state.world, state.get_next_buffer_name("particle"));
    // cout << "float_ min " << FLT_MIN << endl;


    for (int j = 0, len = particles.size(); j < len; ++j) {
        float odo_time_elapsed = odo_state.last_updated - particles[j].t;
        float v_theta = eecs467::angle_diff(particles[j].thetap, particles[j].theta) / odo_time_elapsed;
        float v_x     = (particles[j].xp - particles[j].x) / odo_time_elapsed;
        float v_y     = (particles[j].yp - particles[j].y) / odo_time_elapsed;

        int counts = 0;
        for(int i = 0; i < scan->num_ranges; ++i){
            if(scan->intensities[i] <= 0)
                continue;

            float single_line[4], elapsed_time, x, y;

            elapsed_time = (scan->times[i] - particles[j].t) / 1000000.0;
            x = (scan->ranges[i]) * cosf(scan->thetas[i]);
            y = (scan->ranges[i]) * sinf(scan->thetas[i]);
            rotate_matrix_z(&x, &y, eecs467::angle_sum(particles[j].theta, elapsed_time * v_theta));

            single_line[0] = particles[j].x + elapsed_time * v_x;
            single_line[1] = particles[j].y + elapsed_time * v_y;
            single_line[2] = single_line[0] + x;
            single_line[3] = single_line[1] - y;

            // update weight for particles
            DoublePoint laser_end_position(single_line[2], single_line[3]);
            update_particle_weight(particles[j], laser_end_position);
            
            // Add line to laser buffer
            // add_line_to_buf(laser_buffer, counts < 290/3 ? vx_green : counts < 290*2/3 ? vx_blue : vx_yellow, single_line);
            counts++;
        }
        if (particles[j].weight > max_weight) {
            max_weight = particles[j].weight;
            best_particle_index = j;
        }

        // Add point to ALL PARTICLES
        add_point_to_buf(particles_buffer, vx_red, particles[j].x, particles[j].y, 0.05);
    }


    add_point_to_buf(best_particle_buffer, vx_red, particles[best_particle_index].x, particles[best_particle_index].y, 0.05);
    update_slam_state(particles[best_particle_index], scan->utime);
    state.weight_updated = true;

    // float odo_time_elapsed = odo_state.last_updated - particles[best_particle_index].t;
    // float v_theta = eecs467::angle_diff(particles[best_particle_index].thetap, particles[best_particle_index].theta) / odo_time_elapsed;
    // float v_x     = (particles[best_particle_index].xp - particles[best_particle_index].x) / odo_time_elapsed;
    // float v_y     = (particles[best_particle_index].yp - particles[best_particle_index].y) / odo_time_elapsed;
    // int counts = 0;
    // for(int i = 0; i < scan->num_ranges; ++i){
    //     if(scan->intensities[i] <= 0)
    //         continue;

    //     float single_line[4], elapsed_time, x, y;

    //     elapsed_time = (scan->times[i] - particles[best_particle_index].t) / 1000000.0;
    //     x = (scan->ranges[i]) * cosf(scan->thetas[i]);
    //     y = (scan->ranges[i]) * sinf(scan->thetas[i]);
    //     rotate_matrix_z(&x, &y, eecs467::angle_sum(particles[best_particle_index].theta, elapsed_time * v_theta));

    //     single_line[0] = particles[best_particle_index].x + elapsed_time * v_x;
    //     single_line[1] = particles[best_particle_index].y + elapsed_time * v_y;
    //     single_line[2] = single_line[0] + x;
    //     single_line[3] = single_line[1] - y;

    //     // update weight for particles
    //     DoublePoint laser_end_position(single_line[2], single_line[3]);
    //     update_particle_weight(particles[best_particle_index], laser_end_position);
        
    //     // Add line to laser buffer
    //     add_line_to_buf(laser_buffer, counts < 290/3 ? vx_green : counts < 290*2/3 ? vx_blue : vx_yellow, single_line);
    //     counts++;
    // }

    vx_buffer_swap(particles_buffer);
    vx_buffer_swap(best_particle_buffer);
    // vx_buffer_swap(laser_buffer);
    // cout << "before normalize" << endl;
    // for (int i = 0; i < NUM_PARTICLE; ++i) {
    //     cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
    // }

    //normalize
    double weight_sum = 0;
    for (int j = 0, len = particles.size(); j < len; ++j) {
        double norm_weight = pow (10, particles[j].weight - max_weight); // assume log base 10
        // cout << particles[j].weight << ", " << max_weight << ", " << norm_weight << endl;

        particles[j].weight = norm_weight;
        weight_sum += norm_weight;
    }
    // make sure all sum up to 1
    for (int j = 0, len = particles.size(); j < len; ++j) {
        particles[j].weight /= weight_sum;
    }

    // cout << "after normalize" << endl;
    // for (int i = 0; i < NUM_PARTICLE; ++i) {
    //     cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
    // }

}


void action_model_updater (const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_motor_feedback_t *msg, void *user)
{
    // cout << "doing action model" << endl;

    if (state.weight_updated == false) {
        for (int i = 0; i < NUM_PARTICLE; ++i) {
            particles[i].xp = particles[i].xp + action_state.s*cos(eecs467::angle_sum(particles[i].thetap, action_state.alpha));
            particles[i].yp = particles[i].yp + action_state.s*sin(eecs467::angle_sum(particles[i].thetap, action_state.alpha));
            particles[i].thetap = eecs467::angle_sum(particles[i].thetap, action_state.phi);
        }
    } else {
        // sample
        vector<Particle_t> sampled_particles(NUM_PARTICLE);
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
            sampled_particles[i] = particles[index];
            step_sum += step;

            //cout << "weight_sum " << weight_sum << " step_sum " << step_sum << " index " << index << " i " << i << endl;
        
        }
        // update
        gsl_rng * rng = gslu_rand_rng_alloc();

        float k1 = 1;
        float k2 = 0.4;
        double e1 = 0; 
        double e2 = 0;
        double e3 = 0;
        for (int i = 0; i < NUM_PARTICLE; ++i) {
            e1 = gslu_rand_gaussian(rng, 0, k1*action_state.alpha);
            e2 = gslu_rand_gaussian(rng, 0, k2*action_state.s);
            e3 = gslu_rand_gaussian(rng, 0, k1*(action_state.phi-action_state.alpha));

            float theta_e1 = eecs467::angle_sum(sampled_particles[i].thetap, e1);

            particles[i].x = sampled_particles[i].xp + (action_state.s + e2)*cos(eecs467::angle_sum(theta_e1, action_state.alpha));
            particles[i].y = sampled_particles[i].yp + (action_state.s + e2)*sin(eecs467::angle_sum(theta_e1, action_state.alpha));
            particles[i].theta = eecs467::angle_sum(eecs467::angle_sum(theta_e1, action_state.phi), e3);
            particles[i].xp = particles[i].x;
            particles[i].yp = particles[i].y;
            particles[i].thetap = particles[i].theta;
            particles[i].t = msg->utime;
            particles[i].weight = 1.0/NUM_PARTICLE;
        }
        delete rng;
    }

    state.weight_updated = false;

    vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, "particles");
    for (size_t i = 0; i < particles.size(); i++) {
        add_point_to_buf(buf, vx_red, particles[i].x, particles[i].y, 0.05);
    }
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