struct Particle_t {
    float x;
    float y;
    float theta;
    float weight;
    Particle_t() :x(0), y(0), theta(0), weight(0) {}
    Particle_t(float x_in, float y_in, float t_in, float w_in) : 
    x(x_in), y(y_in), theta(t_in), weight(w_in) {}
};

const int NUM_PARTICLE = 1000;
vector<Particle_t> particles(NUM_PARTICLE, Particle_t(0,0,0,1/num_particle));

std::default_random_engine generator; // Returns unsigned int

void action_model(Particle_t &next_particle) {
    float alpha = action_state.alpha;
    float s = action_state.s;
    float phi = action_state.phi;

    normal_distribution<float> e1_dist(0, 1.0 * alpha);
    normal_distribution<float> e2_dist(0, 0.4 * s);
    normal_distribution<float> e3_dist(0, 1.0 * (phi - alpha));

    unsigned int gen_val = generator;

    float e1 = e1_dist(gen_val);
    float e2 = e2_dist(gen_val);
    float e3 = e3_dist(gen_val);

    next_particle.x += (s + e2)*cos(next_particle.theta + alpha + e1));
    next_particle.y += (s + e2)*sin(next_particle.theta + alpha + e1));
    next_particle.theta += phi + e1 + e3;
}

void particle_filter() {
    vector<Particle_t> new_particles(num_particle);
    
    float sample_weight = 0;
    float weight_counter = 0;
    int sample_counter = 0;
    float weight_step = 1/num_particle;
    
    float total_weight = 0;
    
    int64_t last_updated_time = action_model.last_updated;
    
    for (unsigned int i = 0; i < particles.size(); ++i) {
        // sampling
        while (weight_counter > sample_weight) {
            sample_weight += particles[sample_counter++].weight;
        }
        weight_counter += weight_step;
        Particle_t next_particle = particles[sample_counter];
        
        action_model(next_particle);
         
        float new_weight = 0, elapsed_time = 0;
        
        for (unsigned int j = 0; j < scan->num_ranges; ++j) {
            if(scan->intensities[i] <= 0) {
                continue;
            }
            elapsed_time = 0;
            
            if (scan->times[i] > last_updated_time)
                elapsed_time = scan->times[i] - last_updated_time;
            else
                elapsed_time = -1.0 * (last_updated_time - scan->times[i]);
            elapsed_time /= 1000000.0;
            
            float x, y;
            x = (scan->ranges[j]) * cosf(scan->thetas[j]) + next_particle.x + elapsed_time * action_model.v_x;
            y = (scan->ranges[j]) * sinf(scan->thetas[j]) + next_particle.y + elapsed_time * action_model.v_y;
            rotate_matrix_z(&x, &y, next_particle.theta + elapsed_time * action_model.v_theta);
            
            eecs467::Point<double> p(x, y);
            eecs467::Point<int> cell = global_position_to_grid_cell(p,occupancy_grid_state.grid);
            
            uint8_t grid_value = occupancy_grid_state.grid(cell.x, cell.y);
            
            if (grid_value < BLACK_THRESH) {
                new_weight -= 4;
            }
            else if (grid_value < GREY_THRESH) {
                new_weight -= 12;
            }
            else { // white
                new_weight -= 8;
            }
        }
        
        next_particle.weight = new_weight;
        
        total_weight == new_weight;
        
        new_particles[i] = next_particle;
    }
    
    for (Particle_t &p : particles) {
        p.weight /= total_weight;
    }
    
    particles = new_particles;
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