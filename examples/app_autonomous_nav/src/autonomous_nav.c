/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"

#define DEBUG_MODULE "AUTONOMOUS"
#include "debug.h"


// Log variables
logVarId_t idForwardLL, idForwardML, idForwardMR, idForwardRR;
int16_t forwardLL, forwardML, forwardMR, forwardRR;

logVarId_t idBottomLL, idBottomML, idBottomMR, idBottomRR;
int16_t bottomLL, bottomML, bottomMR, bottomRR;


void getLogIds() {
  idForwardLL = logGetVarId("teensy", "forward_ll");
  idForwardML = logGetVarId("teensy", "forward_ml");
  idForwardMR = logGetVarId("teensy", "forward_mr");
  idForwardRR = logGetVarId("teensy", "forward_rr");
  idBottomLL = logGetVarId("teensy", "bottom_ll");
  idBottomML = logGetVarId("teensy", "bottom_ml");
  idBottomMR = logGetVarId("teensy", "bottom_mr");
  idBottomRR = logGetVarId("teensy", "bottom_rr");
}

void appMain() {
  getLogIds();

  
  while(1) {
    vTaskDelay(M2T(2000));
    int16_t forward_ll = logGetInt(idForwardLL);

    DEBUG_PRINT("Forward ll = %i\n", forward_ll);

  }
}
