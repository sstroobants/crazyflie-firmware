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
#include <math.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "range.h"

#define DEBUG_MODULE "AUTONOMOUS"
#include "debug.h"

#define HOLD_HEIGHT_SCALE 0.015f
#define HOLD_HEIGHT_DEADZONE 0.1f

// Log variables
// sensors
logVarId_t idForwardLL, idForwardML, idForwardMR, idForwardRR;
int16_t forwardLL, forwardML, forwardMR, forwardRR;
logVarId_t idBottomLL, idBottomML, idBottomMR, idBottomRR;
int16_t bottomLL, bottomML, bottomMR, bottomRR;
// states
logVarId_t idHeightEstimate;
float heightEstimate;
// cppm radio channels
logVarId_t idAux3, idCppmRoll, idCppmPitch, idCppmYawrate, idCppmThrust;
float cppmRoll, cppmPitch, cppmYawrate, cppmThrust;
// mode variables
bool isActive = false;
bool setAutonomousMode = false;
bool setManualMode = false;
float holdHeight = 0.0f;

void getLogIds()
{
  idForwardLL = logGetVarId("teensy", "forward_ll");
  idForwardML = logGetVarId("teensy", "forward_ml");
  idForwardMR = logGetVarId("teensy", "forward_mr");
  idForwardRR = logGetVarId("teensy", "forward_rr");
  idBottomLL = logGetVarId("teensy", "bottom_ll");
  idBottomML = logGetVarId("teensy", "bottom_ml");
  idBottomMR = logGetVarId("teensy", "bottom_mr");
  idBottomRR = logGetVarId("teensy", "bottom_rr");
  
  idAux3 = logGetVarId("cppm", "aux3");
  idCppmRoll = logGetVarId("cppm", "roll");
  idCppmPitch = logGetVarId("cppm", "pitch");
  idCppmYawrate = logGetVarId("cppm", "yawrate");
  idCppmThrust = logGetVarId("cppm", "thrust");

  idHeightEstimate = logGetVarId("stateEstimate", "z");

}

static void setHeightHoldSetpoint(setpoint_t *setpoint, float roll, float pitch, float z, float yawrate)
{
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->attitude.roll = roll;
  setpoint->attitude.pitch = pitch;
}

void sendHeightMeasurementToEstimator()
{
  int16_t bottom_ll = logGetInt(idBottomLL);
  int16_t bottom_lr = logGetInt(idBottomML);
  int16_t bottom_mr = logGetInt(idBottomMR);
  int16_t bottom_rr = logGetInt(idBottomRR);

  float bottom_dist = ((float)bottom_ll + (float)bottom_lr + (float)bottom_mr + (float)bottom_rr) / 4;
  bottom_dist = (bottom_dist > 80) ? bottom_dist : 0;
  bottom_dist /= 1000;

  rangeSet(rangeDown, bottom_dist);
  // IMPORTANT: currently no filtering of bottom sensor is performed
  // (except for averaging over all 16 readings) this means that
  // it assumes a flat ground plane, and the estimator has a zero std
  // should be improved later
  rangeEnqueueDownRangeInEstimator(bottom_dist, 0, xTaskGetTickCount());
}

int16_t setActiveStatus()
{
  int16_t autonomousModeSwitch = logGetInt(idAux3);
  // DEBUG_PRINT("aux 3 is currently: %d", autonomous_mode_switch);

  // aux3 = 0       -> transmitter is not connected
  // aux3 = 2000    -> B switch is UP, meaning no autonomous mode
  // aux3 = 1000    -> B switch is DOWN, meaning autonomous mode
  if (autonomousModeSwitch > 0)
  {
    if (autonomousModeSwitch < 1400)
    {
      if (!isActive)
      {
        // switching to autonomous mode
        // DEBUG_PRINT("Switching to autonomous mode\n");
        isActive = true;
        setAutonomousMode = true;
      }
    }
    else if (isActive)
    {
      // DEBUG_PRINT("Switching back to manual mode\n");
      isActive = false;
      setManualMode = true;
    }
  }
  return autonomousModeSwitch;
}

void getCppmSetpoints()
{
  cppmRoll = logGetFloat(idCppmRoll);
  cppmPitch = logGetFloat(idCppmPitch);
  cppmYawrate = logGetFloat(idCppmYawrate);
  cppmThrust = logGetFloat(idCppmThrust);
}

void appMain()
{
  getLogIds();

  while (1)
  {
    // Currently running the app at 50Hz
    vTaskDelay(F2T(50));

    // Get the height measurement from the bottom sensor and push it to the estimator
    sendHeightMeasurementToEstimator();

    // toggle the activate status based on the aux channel
    setActiveStatus();

    // Get the last setpoint and state.
    // Currently only used for maintaining altitude when switching?
    // setpoint_t setpoint;
    // state_t state;
    // commanderGetSetpoint(&setpoint, &state);

    // When we are performing autonomous mode
    if (isActive)
    {
      // If we need to perform switching logic
      if (setAutonomousMode)
      {
        holdHeight = 0.0f;
        heightEstimate = logGetFloat(idHeightEstimate);
        DEBUG_PRINT("Altitude at time of switching: %f\n", (double)heightEstimate);
        holdHeight = heightEstimate;
        setAutonomousMode = false;
      }

      setpoint_t setpoint;
      // state_t state;

      getCppmSetpoints();

      float vz = (cppmThrust - 32767) / 32767.0f;
      vz = (fabsf(vz) < HOLD_HEIGHT_DEADZONE) ? 0.0f : vz;
      holdHeight += vz * HOLD_HEIGHT_SCALE;
      // DEBUG_PRINT("Setpoints: %f, %f, %f, %f\n", (double)cppmRoll, (double)cppmPitch, (double)cppmYawrate, (double)cppmThrust);
      setHeightHoldSetpoint(&setpoint, cppmRoll, cppmPitch, holdHeight, cppmYawrate);
      commanderSetSetpoint(&setpoint, 3);
      DEBUG_PRINT("vz: %f\n", (double) holdHeight);
      // commanderGetSetpoint(&setpoint, &state);
      // DEBUG_PRINT("Setpoints: %f, %f, %f, %f\n", (double)setpoint.attitude.roll, (double)setpoint.attitude.pitch, (double)setpoint.attitudeRate.yaw, (double)setpoint.thrust);
    }
    else if (setManualMode)
    {
      commanderRelaxPriority();
    }
  }
}
