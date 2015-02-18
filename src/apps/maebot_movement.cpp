#include "maebot_movement.h"
#include "maebot_view.h"
#include "find_path.h"

float max_left_speed = 0.25f;
float max_right_speed = 0.25f;

float max_turning_speed = 0.25f;

using namespace std;

/* Control loop: If left ticks is increasing 
more than right ticks, adjust the MAX_SPEED values */

bool rotate_state = true;
int need_publish = 0;
void* send_cmds(void *args)
{
	//Odo_state *state = odo_state;//reinterpret_cast<Odo_state*>(data); // Not sure if correct...

	int32_t starting_left_ticks = odo_state.left;
	int32_t starting_right_ticks = odo_state.right;

	while (1) {
		pthread_mutex_lock(&odo_state.cmd_mutex);

		IntPoint cur_pos = global_position_to_grid_position(DoublePoint(slam_state.x, slam_state.y), expanded_grid);
		if (expanded_grid.logOdds(cur_pos.x, cur_pos.y) == BLACK_VALUE) {
			expanded_grid.setLogOdds(cur_pos.x, cur_pos.y, WHITE_VALUE);
		}


		if (odo_state.movement == 0) {
			double percentage = 0.75;
			odo_state.cmd.motor_left_speed = max_left_speed * percentage;
			odo_state.cmd.motor_right_speed = 0.95 * max_right_speed * percentage;
			
			//cout << "MOVING FORWARD" << endl;
			// Forward

			// Feedback loop
			if (odo_state.left - starting_left_ticks < odo_state.right - starting_right_ticks) {
				odo_state.cmd.motor_left_speed = 0.8*max_left_speed;
				odo_state.cmd.motor_right_speed = 0.7*max_right_speed;
			}
			else if (odo_state.left - starting_left_ticks > odo_state.right - starting_right_ticks) {
				odo_state.cmd.motor_left_speed = 0.7*max_left_speed;
				odo_state.cmd.motor_right_speed = 0.8*max_right_speed;
			}

			if (sqrt(pow(odo_state.start_x - odo_state.x, 2) + pow(odo_state.start_y - odo_state.y, 2)) >= odo_state.distance - 0.1) {
				starting_left_ticks = odo_state.left;
				starting_right_ticks = odo_state.right;
				odo_state.movement = 1;
				// odo_state.distance = 0;
			}
			// system("clear");
			cout << "Moving forward: " << endl;
			cout << "Commanded left speed, right speed = " << odo_state.cmd.motor_left_speed << ", " << odo_state.cmd.motor_right_speed << endl;
			cout << "Target Distance = " << odo_state.distance << ", current distance = " << sqrt(pow(odo_state.start_x - odo_state.x, 2) + pow(odo_state.start_y - odo_state.y, 2)) << endl;
			//odo_state.distance = sqrt(pow(odo_state.dest_x - odo_state.x, 2) + pow(odo_state.dest_y - odo_state.y, 2));
		}
		else if (odo_state.movement == 1) {
			//cout << "STOPPED" << endl;
			// Stopped
			odo_state.cmd.motor_left_speed = 0;
			odo_state.cmd.motor_right_speed = 0;
			starting_left_ticks = odo_state.left;
			starting_right_ticks = odo_state.right;	
			need_publish = 3;
		}
		else if (odo_state.movement == 2) {
			double percentage = 0.60;

			cout << "angle diff = " << eecs467::angle_diff(odo_state.theta, odo_state.angle) << endl;
			// if (eecs467::angle_diff(odo_state.theta, odo_state.angle) > 0) {
				odo_state.cmd.motor_left_speed = percentage * max_turning_speed;
				odo_state.cmd.motor_right_speed = -percentage * max_turning_speed;
			// }
			// else {
			// 	odo_state.cmd.motor_left_speed = -percentage * max_turning_speed;
			// 	odo_state.cmd.motor_right_speed =  percentage * max_turning_speed;
			// }
				
			// system("clear");
			cout << "Rotating: " << endl;
			cout << "Commanded left speed, right speed = " << odo_state.cmd.motor_left_speed << ", " << odo_state.cmd.motor_right_speed << endl;
			cout << "Angle Diff = " << eecs467::angle_diff(odo_state.theta, odo_state.angle) << endl;

			// cout << "before angle diff = " << eecs467::angle_diff(odo_state.theta, odo_state.angle) << endl;
			// state.lcm.publish("MAEBOT_MOTOR_COMMAND", &(odo_state.cmd));
			// odo_state.cmd.motor_left_speed = 0;
			// odo_state.cmd.motor_right_speed = 0;
			// state.lcm.publish("MAEBOT_MOTOR_COMMAND", &(odo_state.cmd));
			// usleep(20000.);
			// cout << "after angle diff = " << eecs467::angle_diff(odo_state.theta, odo_state.angle) << endl;
			//assert(false);
			//cout << "TURNING" << endl;
			if (abs(eecs467::angle_diff(odo_state.theta, odo_state.angle)) < 0.1) {
				odo_state.movement = 1;
				cout << "done turning" << endl;
				odo_state.angle = 0;
				starting_left_ticks = odo_state.left;
				starting_right_ticks = odo_state.right;	
				// assert(false);
			}
		}
		else {
			// If the movement value is anything other than 0, 1, or 2
			odo_state.movement = 1;
		}

		//if (need_publish) {
			//cout << "need_publish :" << need_publish << endl;
			state.lcm.publish("MAEBOT_MOTOR_COMMAND", &(odo_state.cmd));
			// need_publish--;
		//}
		// maebot_motor_command_t_publish(odo_state.lcm, "MAEBOT_MOTOR_COMMAND", &(odo_state.cmd));
		pthread_mutex_unlock(&odo_state.cmd_mutex);
		//out << " sending cmds" << endl;
		//cout << "End of loop" << endl;
		usleep(1000);
	}

	return NULL;
}

void forward(float distance) {
	// while (odo_state.movement != 1) {}
	odo_state.movement = 0;
	odo_state.start_x = odo_state.x;
	odo_state.start_y = odo_state.y;
	odo_state.distance = distance;
	need_publish = 3;
}

void stop() {
	odo_state.movement = 1;
}

void rotate(float angle) {
	// while (odo_state.movement != 1) {}
	odo_state.movement = 2;
	odo_state.angle = eecs467::angle_sum(odo_state.theta, angle);
	need_publish = 5;
}