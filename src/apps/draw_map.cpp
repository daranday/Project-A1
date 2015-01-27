#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>
#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"

// LCM
#include <lcm/lcm.h>
// map
#include "lcmtypes/maebot_occupancy_grid_t.h"


// vx
#include <gtk/gtk.h>
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"
#include "vx/vxo_drawables.h"

#include "imagesource/image_u8.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// map
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"

using namespace std;

const char* app_name = "draw grid map";
const char* occupancy_grid_channel = "OCCUPANCY_GRID";
const int draw_hz = 30;
const int[] default_window_size = {800, 800};

typedef struct state state_t;
struct state
{
   getopt_t *gopt;

// LCM
   lcm_t *lcm;

// vx state
   int running;
   vx_application_t app;
   vx_world_t * world;
   zhash_t * layers;

// thread
   pthread_mutex_t mutex;
   pthread_t draw_thread;

// map
   OccupancyGrid* grid;

};

static void draw(state_t * state, vx_world_t * world)
{
    printf("NTH to draw here\n");
}

static void occupancy_grid_handler(const lcm_recv_buf_t *rbuf,
               const char *channel,
               const maebot_occupancy_grid_t *msg,
               void *user)
{
    state_t* state = (state_t*) user;
    state->grid->fromLCM(msg);

    image_u8_t *im = image_u8_create (state->grid->widthInCells(), state->grid->heightInCells());
    for (int i = 0; i < state->grid->widthInCells(); ++i) {
        for (int j = 0; j < state->grid->heightInCells(); ++j) {
            im->buf[i] = 127 - state->grid->logOdds(i,j);
        }
    }

    if (im != NULL) {

        vx_object_t * vo = vxo_image_from_u8(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);

        vx_buffer_t *vb = vx_world_get_buffer(state->world, "map");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_LEFT,
                                              vxo_chain (vxo_mat_translate3 (0, 0, -0.001),
                                                         vo)));
        vx_buffer_swap(vb);
    }
    else {
        printf("Error converting to image");
    }

    image_u8_destroy(im);
}

static void * read_loop(void * data)
{  
    state_t* state = (state_t*) data;
    int status = 0;
    while (state->running) {
        status = lcm_handle_timeout (state->lcm, 1000/draw_hz);
    }
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
   state_t * state = app->impl;
   pthread_mutex_lock(&state->mutex);

   vx_layer_t * layer = NULL;

   zhash_remove(state->layers, &disp, NULL, &layer);
   vx_layer_destroy(layer);
   pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
   state_t * state = app->impl;

   vx_layer_t * layer = vx_layer_create(state->world);
   vx_layer_set_display(layer, disp);

   pthread_mutex_lock(&state->mutex);
   zhash_put(state->layers, &disp, &layer, NULL, NULL);
   pthread_mutex_unlock(&state->mutex);
}

static state_t * state_create()
{
    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, '\0', "no-gtk", 0, "Don't show gtk window, only advertise remote connection");
    getopt_add_bool (gopt, '\0', "stay-open", 0, "Stay open after gtk exits to continue handling remote connections");

    state_t * state = calloc(1, sizeof(state_t));

    vx_global_init();

    state->gopt = gopt;

    state->lcm = lcm_create (NULL);

    state->running = 1;
    state->app.impl = state;
    state->app.display_started = display_started;
    state->app.display_finished = display_finished;


    state->world = vx_world_create();
    state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    pthread_mutex_init (&state->mutex, NULL);

    state->grid = new OccupancyGrid();

    return state;
}


static void state_destroy(state_t * state)
{
   vx_world_destroy(state->world);
   assert(zhash_size(state->layers) == 0);
   zhash_destroy(state->layers);

   lcm_destroy (state->lcm); // rexarm example

   getopt_destroy (gopt);

   pthread_mutex_destroy(&state->mutex);

   free(state);

   vx_global_destroy(); 
}


int main (int argc, char *argv[]) {    
    state_t * state = state_create(gopt);

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.advertise_name = app_name;
    vx_remote_display_source_t * cxn = vx_remote_display_source_create_attr(&state->app, &remote_attr);

    maebot_occupancy_grid_t_subscribe(state->lcm,
                                        occupancy_grid_channel,
                                        occupancy_grid_handler,
                                        state);

    // draw initial static object
    draw(state, state->world);
    // draw animate object
    pthread_create(&state->draw_thread, NULL, read_loop, state);


    if (!getopt_get_bool(gopt,"no-gtk")) {
        gdk_threads_init ();
        gdk_threads_enter ();

        gtk_init (&argc, &argv);

        vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->app);
        GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
        GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
        // create window
        gtk_window_set_default_size (GTK_WINDOW (window), default_window_size[0], default_window_size[1]);
        gtk_container_add(GTK_CONTAINER(window), canvas);
        gtk_widget_show (window);
        gtk_widget_show (canvas); // XXX Show all causes errors!

        g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

        gtk_main (); // Blocks as long as GTK window is open
        gdk_threads_leave ();

        vx_gtk_display_source_destroy(appwrap);
        // quit when gtk closes? Or wait for remote displays/Ctrl-C
        if (!getopt_get_bool(gopt, "stay-open")) {
            state->running = 0;
        }
    }

    pthread_join(state->draw_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    state_destroy(state);
}
