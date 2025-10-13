// Lightweight storage for swarm telemetry values
#include "swarm_info.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile swarm_info_t g_swarm_info = {0};

void swarmInfoUpdate(float vx, float vy, float vz, float gz, float h) {
  taskENTER_CRITICAL();
  g_swarm_info.vx = vx;
  g_swarm_info.vy = vy;
  g_swarm_info.vz = vz;
  g_swarm_info.gz = gz;
  g_swarm_info.h  = h;
  taskEXIT_CRITICAL();
}

void swarmInfoGet(float* vx, float* vy, float* vz, float* gz, float* h) {
  taskENTER_CRITICAL();
  if (vx) *vx = g_swarm_info.vx;
  if (vy) *vy = g_swarm_info.vy;
  if (vz) *vz = g_swarm_info.vz;
  if (gz) *gz = g_swarm_info.gz;
  if (h)  *h  = g_swarm_info.h;
  taskEXIT_CRITICAL();
}
