#include "maebot_view.h"

#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>

#include "../mapping/occupancy_grid_utils.hpp"

using namespace std;

int main(int argc, char** argv) {
	Maebot_View maebot_world;
	maebot_world.start(argc, argv);
}

