#include "find_path.h"

eecs467::OccupancyGrid expanded_grid;

void init_expanded_grid() {
    expanded_grid = eecs467::OccupancyGrid(grid_width_c, grid_height_c, cell_sides_width_c);
}

void expand_border() {
    for (size_t y = 0; y < state.grid.heightInCells(); ++y) {
        for (size_t x = 0; x < state.grid.widthInCells(); ++x) {
            if (state.grid.logOdds(x,y) > BLACK_THRESH) {
                for (size_t j = y-2; j <= y+2; ++y) {
                    for (size_t i = x-2; i <= x+2; ++x) {
                        if (expanded_grid.isCellInGrid(i,j) && expanded_grid.logOdds(i,j) != BLACK_VALUE) {
                            expanded_grid.setLogOdds(i,j,BLACK_VALUE);
                        }
                    }
                }
            }
            else if (expanded_grid.logOdds(x,y) == GREY_VALUE && state.grid.logOdds(x,y) < WHITE_THRESH) {
                expanded_grid.setLogOdds(x,y,WHITE_VALUE);
            }
        }
    }
}

void back_trace(deque<IntPoint>& path, int end_pos_x, int end_pos_y) {
    IntPoint cur_pos(end_pos_x, end_pos_y);

    while (expanded_grid.logOdds(cur_pos.x, cur_pos.y) != START) {
        path.push_back(cur_pos);
        switch (expanded_grid.logOdds(cur_pos.x, cur_pos.y)) {
            case RIGHT:
                cur_pos.x++;
                break;
            case DOWN:
                cur_pos.y++;
                break;
            case LEFT:
                cur_pos.x--;
                break;
            case UP:
                cur_pos.y--;
                break;expanded_grid
            default:
                cout << "reach unknown" << expanded_grid.logOdds(cur_pos.x, cur_pos.y);
                break;
        }
    }
    path.push_back(cur_pos); // don't push_back start coord
}

bool check_cell(deque<IntPoint>& path, deque<IntPoint>& Q, int x, int y, const int direction) {
    if (expanded_grid.logOdds(x,y) == WHITE_VALUE) {
        expanded_grid.setLogOdds(x,y,direction);
        Q.push_back(IntPoint(x,y));
    }
    else if (expanded_grid.logOdds(x,y) == GREY_VALUE) {
        expanded_grid.setLogOdds(x,y,direction);
        back_trace(path, x, y);
        return true;
    }
    return false;
}

bool grey_BFS(deque<IntPoint>&expanded_grid path) {
    IntPoint start_pos = global_position_to_grid_cell(DoublePoint(slam_state.x, slam_state.y), state.grid);

    expanded_grid.setLogOdds(start_pos.x,start_pos.y, START);

    deque<IntPoint> Q;

    Q.push_back(start_pos);
 
    while (!Q.empty()) {

        IntPoint cur_pos = Q.front();
        Q.pop_back();

        int x = cur_pos.x;
        int y = cur_pos.y;

        if (check_cell(path, Q, x-1, y, RIGHT)) {
            return true;
        }
        if (check_cell(path, Q, x, y-1, DOWN)) {
            return true;
        }
        if (check_cell(path, Q, x+1, y, LEFT)) {
            return true;
        }
        if (check_cell(path, Q, x, y+1, UP)) {
            return true;
        }
    }

    cout << "no grey found" << endl;
    return false;
}

void draw_expanded_map() {

    int w = expanded_grid.widthInCells();
    int h = expanded_grid.heightInCells();


    image_u8_t *im = image_u8_create (w, h);
    // For debugging
    // std::cout << msg->origin_x << "," << msg->origin_y << "," << msg->meters_per_cell << "," << msg->width << "," << msg->height << "," << msg->num_cells << "\n";
    // std::cout << "h " << h << ", w " << w << "\n";
    // std::cout << "stride " << im->stride << "\n";

    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            im->buf[j*im->stride+i] = 127 - expanded_grid.logOdds(i,j);
            //std::cout << (int) im->buf[j*im->stride+i] << ",";
            //std::cout << "buf[" << j*w+i << "]: " << state.grid.logOdds(i,j) << std::endl;
        }
        //std::cout << std::endl;
    }
    // std::cout << "done shifting\n";

    if (im != NULL) {

        vx_object_t * vo = vxo_image_from_u8(im, VXO_IMAGE_NOFLAGS, VX_TEX_MIN_FILTER);

        vx_buffer_t *vb = vx_world_get_buffer(vx_state.world, "expanded_map");
        vx_buffer_add_back(vb,  vxo_chain (
                                    vxo_mat_translate3 (state.scale * (expanded_grid.originInGlobalFrame().x-5), 
                                                        state.scale * expanded_grid.originInGlobalFrame().y, 
                                                        -0.001), 
                                    vxo_mat_scale(state.scale * expanded_grid.metersPerCell()), vo));
        vx_buffer_swap(vb);
    }
    else {
        printf("Error converting to image");
    }
    image_u8_destroy(im);
    // cout << "byegrid" << endl;
}

bool obstacle_trace(double x0, double y0, double x1, double y1) {
    double dx = abs(x1 - x0);
    double dy = abs(y1 - y0);
    double x = x0;
    double y = y0;
    double n = dx + dy - 0.01;
    double x_inc = (x1 > x0) ? 0.01 : -0.01;
    double y_inc = (y1 > y0) ? 0.01 : -0.01;
    double error = dx - dy;
    dx *= 2;
    dy *= 2;


    map<IntPoint, bool> visited;
    for (; n > 0;  n -= 0.01)
    {
        // visit(x, y);

        eecs467::Point<double> p(x, y);
        eecs467::Point<int> cell = global_position_to_grid_cell(p,expanded_grid);
        //cout << "cell.x: " << cell.x << "cell.y: " << cell.y << "(" << x << "," << y << ")" << endl;
        if (visited.find(cell) == visited.end()) {

            if (expanded_grid(cell.x,cell.y) == BLACK_VALUE) {
                return false;
            }
            visited[cell] = true;

            // if (state.grid(cell.x,cell.y) > -128) {       
            //     state.grid(cell.x, cell.y)--;
            // }
            // visited[cell] = true;
        }
        
        if (error > 0)
        {
            x += x_inc;
            error -= dy;
        }

        else
        {
            y += y_inc;
            error += dx;
        }
    }

    return true;

}

void navigate(deque<IntPoint>& path)
{
    int start_point = path.size() - 1
    while (start_point >= 1) {
        int i = 1;
        while (!obstacle_trace(path[start_point].x, path[start_point].y, path[i].x, path[i].y)) {
            i++;
        }
        DoublePoint world_start = DoublePoint(path[start_point].x + cell_sides_width_c / 2,
            path[start_point].y + cell_sides_width_c / 2);
        DoublePoint world_end = DoublePoint(path[i].x + cell_sides_width_c / 2,
            path[i].y + cell_sides_width_c / 2);
        world_start = grid_position_to_global_position(world_start, expanded_grid);
        world_end = grid_position_to_global_position(world_end, expanded_grid);

        float x_diff = world_start.x - world_end.x;
        float y_diff = world_start.y - world_end.y;
        float theta = atan2(y_diff,x_diff);
        float current_theta = slam_state.theta;
        rotate(theta - current_theta, odo_state);
        // Move forward a set distance
        float dist = sqrt((x_diff * x_diff) + (y_diff * y_diff));
        forward(dist, odo_state);
        usleep(200000);

        DoublePoint slam_world_coord = DoublePoint(slam_state.x, slam_state.y);

        IntPoint slam_grid_location = global_position_to_grid_cell(slam_world_coord, expanded_grid);

        path[i].x = slam_grid_location.x;
        path[i].y = slam_grid_location.y;

        // if (fabs(slam_state.x - world_end.x) > 0.1 || fabs(slam_state.y - world_end.y) > 0.1) {

        // }
        for (int j = start_point; j > i; j--) {
            path.pop_back();
        }
    }
}

void movement_control_loop () {
    bool completed = false;
    deque<IntPoint> path;
    init_expanded_grid();

    while (completed == false) {
        expand_border();
        completed = grey_BFS(path);
        draw_expanded_map();
        navigate(path);
    }
}