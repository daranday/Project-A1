// #include "maebot_view.h"
#include "particle_filter.h"
#include "../math/angle_functions.hpp"
#include "../math/gsl_util_rand.h"

#include <vector>

using namespace std;

vector <Particle_t> particles(NUM_PARTICLE);

void update_particle_weight(Particle_t& particle, DoublePoint& laser_end_position) {
    //DoublePoint laser_end_position = DoublePoint(particle.x + delta_r.x, particle.y + delta_r.y);
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


void add_point_to_buf(vx_buffer_t *buf, float x, float y, float z) {
    float current_position[3] = {state.scale * x, state.scale * y, z};
    vx_resc_t *the_point = vx_resc_copyf(current_position, 3);
    vx_object_t *trace = vxo_points(the_point, 1, vxo_points_style(vx_red, 2.0f));
    vx_buffer_add_back(buf, trace);
}

void add_line_to_buf(vx_buffer_t *buf, float x, float y, float z) {

}

void update_slam_state(Particle_t &particle, int64_t last_updated) {
    slam_state.x = particle.x;
    slam_state.y = particle.y;
    slam_state.theta = particle.theta;
    slam_state.last_updated = last_updated;
}

void sensor_model_updater(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_laser_scan_t *scan, void *user)
{
    // cout << "doing sensor model" << endl;

    double max_weight = -std::numeric_limits<double>::max(); // minimum float nubmer
    int best_particle_index;

    vx_buffer_t * mybuf = vx_world_get_buffer(vx_state.world, "Yellow Laser");
    vx_buffer_t * mybuf2 = vx_world_get_buffer(vx_state.world, "current particle");
    // cout << "float_ min " << FLT_MIN << endl;

    for (int j = 0, len = particles.size(); j < len; ++j) {
        int counts = 0;
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
            rotate_matrix_z(&x, &y, eecs467::angle_sum(particles[j].theta, 0.4 * elapsed_time * odo_state.v_theta));

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
            
            // vx_resc_t *verts = vx_resc_copyf(plot_line, 2 * 2);
            // vx_object_t *line = vxo_lines(verts, 2, GL_LINES, vxo_points_style(counts < 290/3 ? vx_green : counts < 290*2/3 ? vx_blue : vx_yellow, 2.0f));
            // vx_buffer_add_back(mybuf, line);
            counts++;
            
            //DoublePoint delta_r(single_line[2] - single_line[0], single_line[3] - single_line[1]);
            DoublePoint laser_end_position(single_line[2], single_line[3]);
            
            update_particle_weight(particles[j], laser_end_position);

        }
        if (particles[j].weight > max_weight) {
            max_weight = particles[j].weight;
            best_particle_index = j;
        }

        // vx_buffer_swap(mybuf);

        
        float current_position[3] = {state.scale * (float)particles[j].x, state.scale * (float)particles[j].y, 0.05};
        vx_resc_t *one_point = vx_resc_copyf(current_position,3);
        vx_object_t *trace = vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)); 
        // vxo_chain(vxo_mat_translate3(state.bot.x, state.bot.y, 0.0),
        // vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
        vx_buffer_add_back(mybuf2, trace);
    }
    vx_buffer_swap(mybuf2);

    char rp_buffer[32];
    sprintf(rp_buffer, "ptc%d", state.particle_counter++);
    vx_buffer_t * particle_trail_buf = vx_world_get_buffer(vx_state.world, rp_buffer);

    add_point_to_buf(vx_world_get_buffer(vx_state.world, rp_buffer), particles[best_particle_index].x, particles[best_particle_index].y, 0.05);
    update_slam_state(particles[best_particle_index], scan->utime);
    vx_buffer_swap(particle_trail_buf);
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

    cout << "after normalize" << endl;
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
    }

}


void action_model_updater (const lcm::ReceiveBuffer* rbuf, const std::string& channel, const maebot_motor_feedback_t *msg, void *user)
{
    // cout << "doing action model" << endl;

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
    vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, "particles");

/*
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        cout << particles[i].x << "," << particles[i].y << ","<< particles[i].theta << "," << particles[i].weight << endl;
        cout << "sample : " << sampled_particles[i] << endl;
    }
*/

    // update
    gsl_rng * rng = gslu_rand_rng_alloc();

    float k1 = 1.0;
    float k2 = 0.4;
    double e1 = 0; 
    double e2 = 0;
    double e3 = 0;
    for (int i = 0; i < NUM_PARTICLE; ++i) {
        e1 = gslu_rand_gaussian(rng, 0, k1*action_state.alpha);
        e2 = gslu_rand_gaussian(rng, 0, k2*action_state.s);
        e3 = gslu_rand_gaussian(rng, 0, k1*(action_state.phi-action_state.alpha));

        float theta_e1 = eecs467::angle_sum(sampled_particles[i].theta, e1);

        particles[i].x = sampled_particles[i].x + (action_state.s + e2)*cos(eecs467::angle_sum(theta_e1, action_state.alpha));
        particles[i].y = sampled_particles[i].y + (action_state.s + e2)*sin(eecs467::angle_sum(theta_e1, action_state.alpha));
        particles[i].theta = eecs467::angle_sum(eecs467::angle_sum(theta_e1, action_state.phi), e3);
        particles[i].weight = 1.0/NUM_PARTICLE;
    }
    delete rng;

    // float current_position[3] = {state.scale * (float)pose_state.x, state.scale * (float)pose_state.y, 0.05};

    for (size_t i = 0; i < particles.size(); i++) {
        add_point_to_buf(buf, particles[i].x, particles[i].y, 0.05);
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