#ifndef __MAEBOT_MOVEMENT__
#define __MAEBOT_MOVEMENT__

void* send_cmds(void *args);
void forward(float distance);
void stop();
void rotate(float angle);

#endif