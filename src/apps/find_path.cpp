#include "find_path.h"
#include "maebot_movement.h"

#include <map>

eecs467::OccupancyGrid expanded_grid;

const float expanded_grid_offset = 15;

void init_expanded_grid() {
    expanded_grid = eecs467::OccupancyGrid(grid_width_c, grid_height_c, cell_sides_width_c);
}

void expand_border() {
    //cout << "in expand border" << endl;
    init_expanded_grid();

    for (size_t y = 0; y < state.grid.heightInCells(); ++y) {
        for (size_t x = 0; x < state.grid.widthInCells(); ++x) {
            if (state.grid.logOdds(x,y) > BLACK_THRESH) {
                //cout << " find black at " << x << "," << y << endl;

                for (size_t j = y-1; j <= y+1; ++j) {
                    for (size_t i = x-1; i <= x+1; ++i) {
                        //cout << "i: " << i << " j: " << j << endl;
                        if (expanded_grid.isCellInGrid(i,j) && expanded_grid.logOdds(i,j) != BLACK_VALUE) {
                            //cout << " setting black at " << i << "," << j << endl;
                            expanded_grid.setLogOdds(i,j,BLACK_VALUE);
                        }
                        else {
                            //cout << " not setting black at " << i << "," << j << endl;
                        }
                    }
                }
            }
            else if (expanded_grid.logOdds(x,y) == GREY_VALUE && state.grid.logOdds(x,y) < WHITE_THRESH) {
                // cout << "putting white at " << x << "," << y << endl;
                expanded_grid.setLogOdds(x,y,WHITE_VALUE);
            }
            /*
            if (x == 100 && y == 100) {
                cout << "log odd of 100 100 " << (int) expanded_grid.logOdds(x,y) << endl;
            }
            */

        }
    }
    /*
    for (size_t y = 0; y < state.grid.heightInCells(); ++y) {
        for (size_t x = 0; x < state.grid.widthInCells(); ++x) {
            cout << (int)expanded_grid.logOdds(x,y) << ",";
        }
        cout << endl;
    }
    */
    //cout << "done expand border" << endl;
}

void back_trace(deque<IntPoint>& path, int end_pos_x, int end_pos_y) {

    cout << "backtracing!" << endl;
    // usleep(3000000);

    vx_buffer_t *path_buf = vx_world_get_buffer(vx_state.world, "path buf");
    
    IntPoint cur_pos(end_pos_x, end_pos_y);

    while (expanded_grid.logOdds(cur_pos.x, cur_pos.y) != START) {
        DoublePoint plan_global = grid_position_to_global_position(DoublePoint(cur_pos.x+1./2, cur_pos.y+1./2) , expanded_grid);

        add_point_to_buf(path_buf, vx_orange, plan_global.x, plan_global.y, 0.05);

        path.push_back(cur_pos);

        //cout << "cur pose " << cur_pos.x << "," << cur_pos.y << endl;

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
                break;
            case START:
                break;
            default:
                cout << "reach unknown" << expanded_grid.logOdds(cur_pos.x, cur_pos.y) << endl;

                vx_buffer_swap(path_buf);
                return;
                break;
        }
    }
    path.push_back(cur_pos); // don't push_back start coord
    DoublePoint plan_global = grid_position_to_global_position(DoublePoint(cur_pos.x+1./2, cur_pos.y+1./2) , expanded_grid);
    add_point_to_buf(path_buf, vx_orange, plan_global.x, plan_global.y, 0.05);

    vx_buffer_swap(path_buf);
}

bool check_cell(deque<IntPoint>& path, deque<IntPoint>& Q, int x, int y, const int direction) {
    //cout << "checking cell " << x << "," << y << " logodd " << (int)expanded_grid.logOdds(x,y) << endl;
    if (expanded_grid.logOdds(x,y) == WHITE_VALUE) {
        expanded_grid.setLogOdds(x,y,direction);
        //cout << "push " << x << "," << y << " logodd " << (int)expanded_grid.logOdds(x,y) << endl;
        if (expanded_grid.logOdds(x-1,y) != BLACK_VALUE && expanded_grid.logOdds(x+1,y) != BLACK_VALUE && 
                expanded_grid.logOdds(x,y-1) != BLACK_VALUE && expanded_grid.logOdds(x,y+1) != BLACK_VALUE &&
                expanded_grid.logOdds(x-1,y-1) != BLACK_VALUE && expanded_grid.logOdds(x-1,y+1) != BLACK_VALUE &&
                expanded_grid.logOdds(x+1,y-1) != BLACK_VALUE && expanded_grid.logOdds(x+1,y+1) != BLACK_VALUE) {
            Q.push_back(IntPoint(x,y));
        }
        
    }
    else if (expanded_grid.logOdds(x,y) == GREY_VALUE) {
        expanded_grid.setLogOdds(x,y,direction);
        back_trace(path, x, y);
        return true;
    }
    return false;
}

bool grey_BFS(deque<IntPoint>&path) {
    cout << "slam " << slam_state.x  << "," << slam_state.y << endl;

    IntPoint start_pos = global_position_to_grid_cell(DoublePoint(slam_state.x, slam_state.y), state.grid);

    expanded_grid.setLogOdds(start_pos.x, start_pos.y, START);

    deque<IntPoint> Q;

    Q.push_back(start_pos);
 
    while (!Q.empty()) {

        IntPoint cur_pos = Q.front();
        Q.pop_front();

        int x = cur_pos.x;
        int y = cur_pos.y;

        //cout << "curpose " << x << "," << y << endl;
        // expanded_grid.setLogOdds(start_pos.x, start_pos.y, 60);

        if (check_cell(path, Q, x+1, y, LEFT)) {
            cout << "grey found at " << x+1 << "," << y << endl;
            return false;
        }
        if (check_cell(path, Q, x, y-1, DOWN)) {
            cout << "grey found at " << x << "," << y-1 << endl;
            return false;
        }
        if (check_cell(path, Q, x-1, y, RIGHT)) {
            cout << "grey found at " << x-1 << "," << y << endl;
            return false;
        }
        if (check_cell(path, Q, x, y+1, UP)) {
            cout << "grey found at " << x << "," << y+1 << endl;
            return false;
        }
        //cout << endl;
    }

    //cout << "no grey found" << endl;
    return true;
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
        // cout << "inside if statement" << endl;
        vx_object_t * vo = vxo_image_from_u8(im, VXO_IMAGE_NOFLAGS, VX_TEX_MIN_FILTER);
        // cout << "created object" << endl;
        vx_buffer_t *vb = vx_world_get_buffer(vx_state.world, "expanded_map");
        // cout << "got buffer" << endl;
        vx_buffer_add_back(vb,  vxo_chain (
                                    vxo_mat_translate3 (state.scale * (expanded_grid.originInGlobalFrame().x), 
                                                        state.scale * expanded_grid.originInGlobalFrame().y, 
                                                        0), 
                                    vxo_mat_scale(state.scale * expanded_grid.metersPerCell()), vo));
        // cout << "add back" << endl;
        vx_buffer_swap(vb);
        // cout << "swap" << endl;
    }
    else {
        printf("Error converting to image");
    }
    image_u8_destroy(im);
    // cout << "destroy" << endl;
    // cout << "byegrid" << endl;
}

bool obstacle_trace(double x0, double y0, double x1, double y1) {

    DoublePoint point0 = DoublePoint(x0 + cell_sides_width_c / 2, y0 + cell_sides_width_c / 2);
    DoublePoint point1 = DoublePoint(x1 + cell_sides_width_c / 2, y1 + cell_sides_width_c / 2);

    point0 = grid_position_to_global_position(point0, expanded_grid);
    point1 = grid_position_to_global_position(point1, expanded_grid);

    double dx = abs(point1.x - point0.x);
    double dy = abs(point1.y - point0.y);
    double x = point0.x;
    double y = point0.y;
    double n = dx + dy - 0.01;
    double x_inc = (point1.x > point0.x) ? 0.01 : -0.01;
    double y_inc = (point1.y > point0.y) ? 0.01 : -0.01;
    double error = dx - dy;
    dx *= 2;
    dy *= 2;

    map<IntPoint, bool> visited;
    for (; n > 0;  n -= 0.01)
    {
        //cout << "in the for loop" << endl;
        // visit(x, y);

        eecs467::Point<double> p(x, y);
        eecs467::Point<int> cell = global_position_to_grid_cell(p,expanded_grid);
        //cout << "cell.x: " << cell.x << "cell.y: " << cell.y << "(" << x << "," << y << ")" << endl;
        //cout << "before if" << endl;
        if (visited.find(cell) == visited.end()) {
            //cout << "inside if" << endl;
            if (expanded_grid(cell.x, cell.y) == BLACK_VALUE) {
                //cout << "inside second if" << endl;
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
    vx_buffer_t *vb = vx_world_get_buffer(vx_state.world, "plan path");

    int start_point = path.size() - 1;
    /*
    for (int i = 0; i < path.size(); i++) {
        cout << "point: " << i << ": " << path[i].x << ", " << path[i].y << endl;
    }
    */




    //while (start_point > 2) {
        int i = 2;
        if (path.size() <= 3) {
            i = 1;

        }

        //cout << "before obstacle_trace" << endl;
        int count = 4;
        while (!obstacle_trace(path[start_point].x, path[start_point].y, path[i].x, path[i].y)) {
            i++;
        }
        while (i < start_point && count && obstacle_trace(path[start_point].x, path[start_point].y, path[i].x, path[i].y)) {
            count--;
            i++;
        }
        cout << " start_point " << start_point << " i " << i << endl;

        // DoublePoint point0 = DoublePoint(path[start_point].x + cell_sides_width_c / 2, path[start_point].y + cell_sides_width_c / 2);
        // DoublePoint point1 = DoublePoint(path[i].x + cell_sides_width_c / 2, path[i].y + cell_sides_width_c / 2);

        // point0 = grid_position_to_global_position(point0, expanded_grid);
        // point1 = grid_position_to_global_position(point1, expanded_grid);
        
        //cout << "after obstacle_trace" << endl;
        DoublePoint world_start = DoublePoint(path[start_point].x + 0.5,
            path[start_point].y + 0.5);
        DoublePoint world_end = DoublePoint(path[i].x + 0.5,
            path[i].y + 0.5);
        world_start = grid_position_to_global_position(world_start, expanded_grid);
        world_end = grid_position_to_global_position(world_end, expanded_grid);


        // while(1) {
        //     rotate(3.14);
        //     usleep(1000000);
        //     cout << "rotate pi" << endl;
        // }

        // while(1) {
        //     // cout << "forward" << endl;
        //     // while (odo_state.movement != 1) {}
        //     // forward(0.5);
        //     // usleep(3000000);
        //     cout << "rotate" << endl;
        //     while (odo_state.movement != 1) {}
        //     rotate(3.14);
        //     usleep(3000000);
        //     // cout << "forward, rotate" << endl;
        // }

        float x_diff = world_end.x - world_start.x;
        float y_diff = world_end.y - world_start.y;
        float theta = atan2(y_diff,x_diff);
        float current_theta = slam_state.theta;
        
        while (odo_state.movement != 1) {
            usleep(10000.);
        }

        if (fabs(eecs467::angle_diff(theta, current_theta)) > 0.0001)  {
            cout << "going to rotate " << eecs467::angle_diff(theta, current_theta) << endl;
            rotate(eecs467::angle_diff(theta, current_theta));
            while (odo_state.movement != 1) {
                usleep(10000.);
            }
        }

        float line[4] = {(float) odo_state.x, (float) odo_state.y, (float) world_end.x, (float) world_end.y};
        add_line_to_buf(vb, vx_green, line);
        vx_buffer_swap(vb);
        

        // Move forward a set distance
        float dist = 1.2*sqrt(pow(odo_state.x - world_end.x, 2) + pow(odo_state.y - world_end.y, 2));
        //float dist = 0.1;
        cout << "dist: " << dist << endl;
        
        if (dist > 0.0001) {
            cout << "going to forward " << dist << endl;
            forward(dist);
        }

        while (odo_state.movement != 1) {
            usleep(10000.);
        }

        cout << "finish moving" << endl;

        DoublePoint slam_world_coord = DoublePoint(slam_state.x, slam_state.y);

        IntPoint slam_grid_location = global_position_to_grid_cell(slam_world_coord, expanded_grid);

        //path[i].x = slam_grid_location.x;
        //path[i].y = slam_grid_location.y;

        // if (fabs(slam_state.x - world_end.x) > 0.1 || fabs(slam_state.y - world_end.y) > 0.1) {

        // }
        /*
        for (int j = start_point; j > i; j--) {
            path.pop_back();
        }
        */
        //start_point = i;
   // }

    //return true;
}

void* pathfinding_loop(void* args) {
    cout << "in path find" << endl;

    bool completed = false;
    

    //cout << "before init" << endl;

    init_expanded_grid();

    //cout << "before while" << endl;

    while (completed == false) {
        deque<IntPoint> path;
        usleep(3000000);
        //cout << "I slept" << endl;
        expand_border();
        cout << "expand_border" << endl;
        draw_expanded_map();
        cout << "draw_expanded_map" << endl;
        completed = grey_BFS(path);
        cout << "grey_BFS completed = " << completed << endl;
        if (completed)
            break;
        navigate(path);
        cout << "navigate" << endl;
        
        vx_buffer_t *vb = vx_world_get_buffer(vx_state.world, "expanded_map");
        vx_buffer_swap(vb);
        usleep(2000000);
    }
    
    cout << "path find done" << endl;

    char k;
    cin >> k;

    return NULL;
}