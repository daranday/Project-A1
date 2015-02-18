#include "maebot_movement.h"
#include "maebot_view.h"

float max_left_speed = 1.0f;
float max_right_speed = 1.0f;

using namespace std;

/* Control loop: If left ticks is increasing 
more than right ticks, adjust the MAX_SPEED values */

void* send_cmds(void *args)
{
	//Odo_state *state = odo_state;//reinterpret_cast<Odo_state*>(data); // Not sure if correct...

	int32_t starting_left_ticks = odo_state.left;
	int32_t starting_right_ticks = odo_state.right;

	while (1) {
		pthread_mutex_lock(&odo_state.cmd_mutex);

		if (odo_state.movement == 0) {
			// Forward
			odo_state.cmd.motor_left_speed = max_left_speed;
			odo_state.cmd.motor_right_speed = max_right_speed;

			// Feedback loop
			if (odo_state.left - starting_left_ticks < odo_state.right - starting_right_ticks) {
				max_left_speed += 0.005;
			}
			else if (odo_state.left - starting_left_ticks > odo_state.right - starting_right_ticks) {
				max_left_speed -= 0.005;
			}
			if (odo_state.left - starting_left_ticks >= odo_state.distance / 0.0002) {
				odo_state.movement = 1;
				odo_state.distance = 0;
				starting_left_ticks = odo_state.left;
				starting_right_ticks = odo_state.right;	
			}
		}
		else if (odo_state.movement == 1) {
			// Stopped
			odo_state.cmd.motor_left_speed = 0;
			odo_state.cmd.motor_right_speed = 0;
		}
		else if (odo_state.movement == 2) {
			if (odo_state.angle > 0) {
				odo_state.cmd.motor_left_speed = max_left_speed;
				odo_state.cmd.motor_right_speed = -max_right_speed;
				if (odo_state.left - starting_left_ticks >= 3.33 * odo_state.angle) {
					odo_state.movement = 1;
					odo_state.angle = 0;
				}
			}
			else if (odo_state.angle < 0) {
				odo_state.cmd.motor_left_speed = -max_left_speed;
				odo_state.cmd.motor_right_speed = max_right_speed;
				if (starting_right_ticks - odo_state.right <= 3.33 * odo_state.angle) {
					odo_state.movement = 1;
					odo_state.angle = 0;
				}
			}
			else {
				// If the angle is 0
				odo_state.movement = 1;
			}
		}
		else {
			// If the movement value is anything other than 0, 1, or 2
			odo_state.movement = 1;
		}

		state.lcm.publish("MAEBOT_MOTOR_COMMAND", &(odo_state.cmd));
		// maebot_motor_command_t_publish(odo_state.lcm, "MAEBOT_MOTOR_COMMAND", &(odo_state.cmd));
		pthread_mutex_unlock(&odo_state.cmd_mutex);
	}

	return NULL;
}

void forward(float distance) {
	odo_state.movement = 0;
	odo_state.distance = distance;
}

void stop() {
	odo_state.movement = 1;
}

void rotate(float angle) {
	odo_state.movement = 2;
	odo_state.angle = angle;
}