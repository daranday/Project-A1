

#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

float max_left_speed = 1.0f;
float max_right_speed = 1.0f;

using namespace std;

/* Control loop: If left ticks is increasing 
more than right ticks, adjust the MAX_SPEED values */

static void* send_cmds(void *data)
{
	Odo_state *state = data;

	int32_t starting_left_ticks = state->left;
	int32_t starting_right_ticks = state->right;

	while (state->running) {
		pthread_mutex_lock(&state->cmd_mutex);

		if (state->movement == 0) {
			// Forward
			state->cmd.motor_left_speed = max_left_speed;
			state->cmd.motor_right_speed = max_right_speed;

			// Feedback loop
			if (state->left - starting_left_ticks < state->right - starting_right_ticks) {
				max_left_speed += 0.05;
			}
			else if (state->left - starting_left_ticks > state->right - starting_right_ticks) {
				max_left_speed -= 0.05;
			}

			starting_left_ticks = state->left;
			starting_right_ticks = state->right;
		}
		else if (state->movement == 1) {
			// Stopped
			state->cmd.motor_left_speed = 0;
			state->cmd.motor_right_speed = 0;
		}
		else if (state->movement == 2) {
			if (state->angle > 0) {
				state->cmd.motor_left_speed = max_left_speed;
				state->cmd.motor_right_speed = -max_right_speed;
				if (state->left - starting_left_ticks >= 3.33 * state->angle) {
					state->movement = 0;
					state->angle = 0;
				}
			}
			else if (state->angle < 0) {
				state->cmd.motor_left_speed = -max_left_speed;
				state->cmd.motor_right_speed = max_right_speed;
				if (starting_right_ticks - state->right <= 3.33 * state->angle) {
					state->movement = 0;
					state->angle = 0;
				}
			}
			else {
				// If the angle is 0
				state->movement = 1;
			}
		}
		else {
			// If the movement value is anything other than 0, 1, or 2
			state->movement = 1;
		}

		maebot_motor_command_t_publish(state->lcm, "MAEBOT_MOTOR_COMMAND", &(state->cmd));
		pthread_mutex_unlock(&state->cmd_mutex);
	}

	return NULL;
}

void forward(void *data) {
	Odo_state *state = data;
	state->movement = 0;
}

void stop(void *data) {
	Odo_state *state = data;
	state->movement = 1;
}

void rotate(float angle, void *data) {
	Odo_state *state = data;
	state->movement = 2;
	state->angle = angle;
}