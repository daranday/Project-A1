#ifndef __FIND_PATH__
#define __FIND_PATH__

#include "maebot_view.h"
#include <queue>

using namespace std;

typedef eecs467::Point<int> IntPoint;
typedef eecs467::Point<double> DoublePoint;

extern eecs467::OccupancyGrid expanded_grid;

const int LEFT = 4;
const int RIGHT = 6;
const int UP = 8;
const int DOWN = 2;
const int START = 5;

const int BLACK_THRESH = 64;
const int WHITE_THRESH = -42;
const int BLACK_VALUE = 127;
const int WHITE_VALUE = -128;
const int GREY_VALUE = 0;

void init_expanded_grid();
void expand_border();
void grey_BFS(queue<IntPoint>& path);

#endif