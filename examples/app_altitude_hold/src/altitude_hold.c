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
#include "platform_defaults.h"
#include "led.h"
#include "ledseq.h"

#define DEBUG_MODULE "AUTONOMOUS"
#include "debug.h"

// Log variables
// cppm radio channels
logVarId_t idAux1, idAux2, idAux3, idCppmRoll, idCppmPitch, idCppmYawrate, idCppmThrust;
float cppmRoll, cppmPitch, cppmYawrate, cppmThrust;
// states
logVarId_t idHeightEstimate;
float heightEstimate;

// Param variables
paramVarId_t idCanLog;
paramVarId_t idLogEnabled;

// mode variables
bool isActive = false;
bool setAutonomousMode = false;
bool setManualMode = false;
float holdHeight = 0.0f;
float holdHeightDeadzone = AUTNAV_HOLD_HEIGHT_DEADZONE;
float holdHeightScale = AUTNAV_HOLD_HEIGHT_SCALE;


void getLogIds()
{
  idAux1 = logGetVarId("cppm", "aux1");
  idAux2 = logGetVarId("cppm", "aux2");
  idAux3 = logGetVarId("cppm", "aux3");
  idCppmRoll = logGetVarId("cppm", "roll");
  idCppmPitch = logGetVarId("cppm", "pitch");
  idCppmYawrate = logGetVarId("cppm", "yawrate");
  idCppmThrust = logGetVarId("cppm", "thrust");

  idHeightEstimate = logGetVarId("stateEstimate", "z");
  
  idCanLog = paramGetVarId("usd", "canLog");
  idLogEnabled = paramGetVarId("usd", "logging");

}

static void setHeightHoldSetpoint(setpoint_t *setpoint, float roll, float pitch, float z, float yawrate)
{
  setpoint->mode.x = modeVelocity;
  setpoint->velocity.x = pitch * -0.03f;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.y = roll * -0.03f;
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
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

void toggleLogging()
{
  if (logGetUint(idAux1) > 0) { // only toggle if remote is connected
    if (paramGetUint(idCanLog) && !paramGetUint(idLogEnabled) && (logGetUint(idAux1) < 1400))
    {
      paramSetInt(idLogEnabled, 1);
    }
    else if (paramGetUint(idLogEnabled) && (logGetUint(idAux1) > 1400))
    {
      paramSetInt(idLogEnabled, 0);
    }
  }
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

    // toggle the activate status based on the aux channel
    setActiveStatus();

    toggleLogging();

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
      vz = (fabsf(vz) < holdHeightDeadzone) ? 0.0f : vz;
      holdHeight += vz * holdHeightScale;

      DEBUG_PRINT("cppmRoll: %f, cppmPitch: %f, holdHeight: %f, cppmYawrate: %f\n", (double)cppmRoll, (double)cppmPitch, (double)holdHeight, (double)cppmYawrate);

      setHeightHoldSetpoint(&setpoint, cppmRoll, cppmPitch, holdHeight, cppmYawrate);
      commanderSetSetpoint(&setpoint, 3);
      // commanderGetSetpoint(&setpoint, &state);
      // DEBUG_PRINT("Setpoints: %f, %f, %f, %f\n", (double)setpoint.attitude.roll, (double)setpoint.attitude.pitch, (double)setpoint.attitudeRate.yaw, (double)setpoint.thrust);
    }
    else if (setManualMode)
    {
      commanderRelaxPriority();
      setManualMode = false;
    }
  }
}



PARAM_GROUP_START(alt_hold)
/**
 * @brief Scale for height hold setpoint based on thrust
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, holdHghtScale, &holdHeightScale)
/**
 * @brief Deadzone for height hold setpoint
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, holdHghtDeadzone, &holdHeightDeadzone)
PARAM_GROUP_STOP(alt_hold)