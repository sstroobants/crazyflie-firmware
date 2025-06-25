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

// #define HOLD_HEIGHT_SCALE 0.015f
// #define HOLD_HEIGHT_DEADZONE 0.15f
// #define OBSTACLE_MINIMUM_DISTANCE 1.0f // meters
// #define AVOID_DURATION 800 // milliseconds
// #define AVOID_PITCH 1.0f
// #define FORWARD_PITCH -15.0f  
// #define AVOID_YAWRATE 60.0f

// Log variables
// sensors
logVarId_t idForwardLL, idForwardML, idForwardMR, idForwardRR;
float forwardLL, forwardML, forwardMR, forwardRR;
logVarId_t idBottomLL, idBottomML, idBottomMR, idBottomRR;
logVarId_t idTeensyStatus;
// states
logVarId_t idHeightEstimate;
float heightEstimate;
// cppm radio channels
logVarId_t idAux3, idCppmRoll, idCppmPitch, idCppmYawrate, idCppmThrust;
float cppmRoll, cppmPitch, cppmYawrate, cppmThrust;

// height variables
int16_t bottom_ll_prev = 0;

// mode variables
bool isActive = false;
bool setAutonomousMode = false;
bool setManualMode = false;
bool avoiding = false;
float holdHeight = 0.0f;
uint32_t startAvoidingTime;
float yawRateOffset = 0.0f;
float pitchOffset = 0.0f;

float forwardPitch = AUTNAV_FORWARD_PITCH; // pitch offset to apply when avoiding obstacles
float avoidPitch = AUTNAV_AVOID_PITCH;
float obstacleMinimumDistance = AUTNAV_OBSTACLE_MINIMUM_DISTANCE; // minimum distance to an obstacle to trigger avoidance
float avoidYawrate = AUTNAV_AVOID_YAWRATE; // yaw rate to apply when avoiding obstacles
uint32_t avoidDuration = AUTNAV_AVOID_DURATION; // duration of the avoidance maneuver in milliseconds
float holdHeightScale = AUTNAV_HOLD_HEIGHT_SCALE; // scale for the height hold setpoint based on thrust
float holdHeightDeadzone = AUTNAV_HOLD_HEIGHT_DEADZONE; // deadzone for the height hold setpoint

// LED variables
ledseqStep_t seq_dist_left_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(200)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_dist_left = {
  .sequence = seq_dist_left_def,
  .led = LED_BLUE_NRF,
};

ledseqStep_t seq_dist_right_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(200)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_dist_right = {
  .sequence = seq_dist_right_def,
  .led = LED_BLUE_L,
};


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
  idTeensyStatus = logGetVarId("teensy", "status");
  
  idAux3 = logGetVarId("cppm", "aux3");
  idCppmRoll = logGetVarId("cppm", "roll");
  idCppmPitch = logGetVarId("cppm", "pitch");
  idCppmYawrate = logGetVarId("cppm", "yawrate");
  idCppmThrust = logGetVarId("cppm", "thrust");

  idHeightEstimate = logGetVarId("stateEstimate", "z");

}

void turnOffLeds()
{
  // Turn off all LEDs
  ledSet(LED_BLUE_L, 0);
  ledSet(LED_BLUE_NRF, 0);
}

void turnOnLeds()
{
  // Turn on all LEDs
  ledSet(LED_BLUE_L, 1);
  ledSet(LED_BLUE_NRF, 1);
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
  int16_t bottom_ml = logGetInt(idBottomML);
  int16_t bottom_mr = logGetInt(idBottomMR);
  int16_t bottom_rr = logGetInt(idBottomRR);

  if (bottom_ll_prev != bottom_ll)
  {
    float bottom_dist = ((float)bottom_ll + (float)bottom_ml + (float)bottom_mr + (float)bottom_rr) / 4;
    bottom_dist = (bottom_dist > 80) ? bottom_dist : 0;
    bottom_dist /= 1000;
  
    rangeSet(rangeDown, bottom_dist);
    
    // IMPORTANT: currently no filtering of bottom sensor is performed
    // (except for averaging over all readings) this means that
    // it assumes a flat ground plane
    rangeEnqueueDownRangeInEstimator(bottom_dist, 0, xTaskGetTickCount());
  }
  bottom_ll_prev = bottom_ll;
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

bool avoidForwardObstacles()
{
  int16_t forward_ll = logGetInt(idForwardLL);
  int16_t forward_ml = logGetInt(idForwardML);
  int16_t forward_mr = logGetInt(idForwardMR);
  int16_t forward_rr = logGetInt(idForwardRR);

  forwardLL = forward_ll > 0 ? (float) forward_ll / 1000.0f: 4.0f;
  forwardML = forward_ml > 0 ? (float) forward_ml / 1000.0f: 4.0f;
  forwardMR = forward_mr > 0 ? (float) forward_mr / 1000.0f: 4.0f;
  forwardRR = forward_rr > 0 ? (float) forward_rr / 1000.0f: 4.0f;
  
  if (forwardML < obstacleMinimumDistance || forwardMR < obstacleMinimumDistance)
  {
    // If any of the forward sensors is below certain distance, we need to stop
    return true;
  }
  else
  {
    // DEBUG_PRINT("No obstacles detected, continuing\n");
    return false;
  }
}

void appMain()
{
  getLogIds();

  // Register the LED sequence
  ledseqRegisterSequence(&seq_dist_left);
  ledseqRegisterSequence(&seq_dist_right);

  while (1)
  {
    // Currently running the app at 50Hz
    vTaskDelay(F2T(50));

    // // Get the forward distances and print them
    // avoidForwardObstacles();

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
      if (!logGetUint(idTeensyStatus)){
        DEBUG_PRINT("Teensy is not responding, stopping autonomous mode\n");
        isActive = false;
        setManualMode = true;
        continue; // Skip the rest of the loop if Teensy is not responding
      }
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

      // Pitch 6 degrees forward always, unless we are avoiding obstacles
      pitchOffset = forwardPitch;

      bool startAvoiding = avoidForwardObstacles();

      if (!avoiding && startAvoiding)
      {
        avoiding = true;
        startAvoidingTime = T2M(xTaskGetTickCount());
        DEBUG_PRINT("Obstacle detected, stopping!\n");
        yawRateOffset = (forwardLL > forwardRR) ? avoidYawrate : -avoidYawrate;
        turnOffLeds();
      }

      if (avoiding)
      {
        // If we are avoiding obstacles, we pitch back a bit and yaw towards "most free" side
        pitchOffset = avoidPitch;
        uint32_t currentAvoidingTime = T2M(xTaskGetTickCount());
        if (currentAvoidingTime - startAvoidingTime > avoidDuration) {
          avoiding = false;
          yawRateOffset = 0.0f; // Reset yaw rate offset
          DEBUG_PRINT("Avoiding finished, resuming normal flight\n");
          turnOnLeds();
        }
      }

      cppmPitch += pitchOffset;
      cppmYawrate += yawRateOffset;

      // DEBUG_PRINT("cppmRoll: %f, cppmPitch: %f, holdHeight: %f, cppmYawrate: %f\n", (double)cppmRoll, (double)cppmPitch, (double)holdHeight, (double)cppmYawrate);

      setHeightHoldSetpoint(&setpoint, cppmRoll, cppmPitch, holdHeight, cppmYawrate);
      commanderSetSetpoint(&setpoint, 3);
      DEBUG_PRINT("vz: %f\n", (double) holdHeight);
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



PARAM_GROUP_START(auto_nav)

/**
 * @brief Pitch offset to apply when moving forward
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, forwardPitch, &forwardPitch)
/**
 * @brief Pitch offset to apply when avoiding obstacles
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, avoidPitch, &avoidPitch)
/**
 * @brief Duration of avoidance maneuver in milliseconds
 */
PARAM_ADD(PARAM_UINT32 | PARAM_PERSISTENT, avoidDuration, &avoidDuration)
/**
 * @brief Minimum distance to obstacle to trigger avoidance (meters)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, obstMinDistance, &obstacleMinimumDistance)
/**
 * @brief Yaw rate to apply when avoiding obstacles (deg/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, avoidYawrate, &avoidYawrate)
/**
 * @brief Scale for height hold setpoint based on thrust
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, holdHghtScale, &holdHeightScale)
/**
 * @brief Deadzone for height hold setpoint
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, holdHghtDeadzone, &holdHeightDeadzone)
PARAM_GROUP_STOP(auto_nav)