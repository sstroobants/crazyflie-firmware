#include <stdint.h>

static int call_count = 0;

uint32_t xTaskGetTickCount()
{
  // Always return 0 for deterministic tests
  call_count++;
  return 0;
}

void vTaskDelay(uint32_t delay) {
  return;
}