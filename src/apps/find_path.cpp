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

void back_trace(queue<IntPoint>& path, int end_pos_x, int end_pos_y) {
    IntPoint cur_pos(end_pos_x, end_pos_y);

    while (expanded_grid.logOdds(cur_pos.x, cur_pos.y) != START) {
        path.push(cur_pos);
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
            default:
                cout << "reach unknown" << expanded_grid.logOdds(cur_pos.x, cur_pos.y);
                break;
        }
    }
    //path.push(cur_pos); // don't push start coord
}

bool check_cell(queue<IntPoint>& path, queue<IntPoint>& Q, int x, int y, const int direction) {
    if (expanded_grid.logOdds(x,y) == WHITE_VALUE) {
        expanded_grid.setLogOdds(x,y,direction);
        Q.push(IntPoint(x,y));
    }
    else if (expanded_grid.logOdds(x,y) == GREY_VALUE) {
        expanded_grid.setLogOdds(x,y,direction);
        back_trace(path, x, y);
        return true;
    }
    return false;
}

void grey_BFS(queue<IntPoint>& path) {
    IntPoint start_pos = global_position_to_grid_cell(DoublePoint(slam_state.x, slam_state.y), state.grid);

    expanded_grid.setLogOdds(start_pos.x,start_pos.y, START);

    queue<IntPoint> Q;

    Q.push(start_pos);
 
    while (!Q.empty()) {

        IntPoint cur_pos = Q.front();
        Q.pop();

        int x = cur_pos.x;
        int y = cur_pos.y;

        if (check_cell(path, Q, x-1, y, RIGHT)) {
            return;
        }
        if (check_cell(path, Q, x, y-1, DOWN)) {
            return;
        }
        if (check_cell(path, Q, x+1, y, LEFT)) {
            return;
        }
        if (check_cell(path, Q, x, y+1, UP)) {
            return;
        }
    }

    cout << "no grey found" << endl;
    return;
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