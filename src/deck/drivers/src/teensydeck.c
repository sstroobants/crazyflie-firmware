/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
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
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "TEENSY"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "uart1.h"
#include "teensydeck.h"

#include "cf_math.h"

static bool isInit = false;
bool status = false;


//////////////COMMUNICATION VARIABLES///////////////////
#define START_BYTE_SERIAL_CF 0x9A

struct serial_pid_in myserial_pid_in;
struct serial_pid_out myserial_pid_out;
uint8_t serial_cf_msg_buf_out[ 2*sizeof(struct serial_pid_out) ] = {0};
uint16_t serial_cf_buf_out_cnt = 0;
int serial_cf_received_packets = 0;

bool receiving;
bool sending;

/////////////INTERNAL LOG VARIABLES
logVarId_t idThrust;
float thrust;
logVarId_t idGyroX, idGyroY, idGyroZ;
float gyroX, gyroY, gyroZ;
logVarId_t idControllerRollRate, idControllerPitchRate, idControllerYawRate;
float controllerRollRate, controllerPitchRate, controllerYawRate;


void serialParseMessageOut(void)
{
  //Copy received buffer to structure
  memmove(&myserial_pid_out,&serial_cf_msg_buf_out[1],sizeof(struct serial_pid_out)-1);
//   DEBUG_PRINT("Correct message received and storing\n");
//   DEBUG_PRINT("Stored roll p, i, d is %i, %i, %i\n", myserial_pid_out.roll_p, myserial_pid_out.roll_i, myserial_pid_out.roll_d);
}

void setPidInMessage(void) 
{
    // Get roll input values and put them in the message
    thrust = logGetFloat(idThrust);
    gyroX = logGetFloat(idGyroX);
    gyroY = logGetFloat(idGyroY);
    controllerRollRate = logGetFloat(idControllerRollRate);
    controllerPitchRate = logGetFloat(idControllerPitchRate);
    myserial_pid_in.thrust = thrust;
    myserial_pid_in.roll = gyroX;
    myserial_pid_in.pitch = gyroY;
    myserial_pid_in.roll_t = controllerRollRate;
    myserial_pid_in.pitch_t = controllerPitchRate;
}

// Read a pid out message over uart
void uartReadPidOutMessage(void) 
{
    uint8_t serial_cf_byte_in;
    // uart1Getchar(&serial_cf_byte_in);
    if (!uart1GetDataWithTimeout(&serial_cf_byte_in, 100)) {
        receiving = false;
        // if status was true, set it to false for debugging purposes
        if (status) {
            DEBUG_PRINT("Did not receive message on time, trying to resend\n");
            status = false;
        }
    };

    if ((serial_cf_byte_in == START_BYTE_SERIAL_CF) || (serial_cf_buf_out_cnt > 0)) {
        serial_cf_msg_buf_out[serial_cf_buf_out_cnt] = serial_cf_byte_in;
        serial_cf_buf_out_cnt++;
    }
    if (serial_cf_buf_out_cnt > sizeof(struct serial_pid_out)  ) {
        serial_cf_buf_out_cnt = 0;
        uint8_t checksum_in_local = 0;
        for(uint16_t i = 1; i < sizeof(struct serial_pid_out) ; i++){
            checksum_in_local += serial_cf_msg_buf_out[i];
        }
        if(checksum_in_local == serial_cf_msg_buf_out[sizeof(struct serial_pid_out)]){
            serialParseMessageOut();
            serial_cf_received_packets++;
            // if status was false, set it to true for debugging purposes
            if (!status) {
                DEBUG_PRINT("Connection (re-)gained\n");
                status = true;
            }
        }
        else {
            DEBUG_PRINT("Incorrect message\n");
        }
        // receiving done; set to false
        receiving = false;
    }
}

// Send a PidIn message via uart to Teensy
void uartSendPidInMessage(void)
{
    //Calculate checksum for outbound packet: 
    uint8_t *buf_send = (uint8_t *)&myserial_pid_in;
    myserial_pid_in.checksum_in = 0;
    for(uint16_t i = 0; i < sizeof(struct serial_pid_in) - 1; i++){
        myserial_pid_in.checksum_in += buf_send [i];
    }
    uint8_t startByte = START_BYTE_SERIAL_CF;
    uart1SendDataDmaBlocking(1, &startByte);
    uart1SendDataDmaBlocking(sizeof(struct serial_pid_in), buf_send);
    // set sending is false after message is sent
    sending = false;
    receiving = true;
    // DEBUG_PRINT("Just sent data\n");
}

void teensyInit(DeckInfo* info)
{
  if (isInit)
    return;
  // initialize connection with the Teensy
  // at baudrate 115200, 500hz not achieved
  // at baudrate 460800, seems to work
  // at baudrate 921600, module seems to crash
  uart1Init(460800);


  // get the logVarIds that are used to get the state/target info
  idGyroX = logGetVarId("gyro", "x");
  idGyroY = logGetVarId("gyro", "y");
  idGyroZ = logGetVarId("gyro", "z");
  idThrust = logGetVarId("controller", "actuatorThrust");
  idControllerRollRate = logGetVarId("controller", "rollRate");
  idControllerPitchRate = logGetVarId("controller", "pitchRate");
  idControllerYawRate = logGetVarId("controller", "yawRate");

  xTaskCreate(teensyTask, TEENSY_TASK_NAME, TEENSY_TASK_STACKSIZE, NULL, TEENSY_TASK_PRI, NULL);

  DEBUG_PRINT("FINISHED CREATING TASK\n");
  status = true;
  isInit = true;
}

bool teensyTest(void)
{
  if (!isInit)
    return false;
  else
    return true;
}

void teensyTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  TickType_t xLastDebugTime;
  xLastDebugTime = T2M(xTaskGetTickCount());

  while (1) {
    if (sending) {
        setPidInMessage();
        uartSendPidInMessage();
    } else if (receiving) {
        uartReadPidOutMessage();
    } else {
        vTaskDelayUntil(&xLastWakeTime, M2T(2));
        sending = true;
    }
    // Printing the amount of received messages over the last seconds
    uint32_t now_ms = T2M(xTaskGetTickCount());
    if (now_ms - xLastDebugTime > 1000) {
        DEBUG_PRINT("received %i messages in the last second\n", serial_cf_received_packets);
        serial_cf_received_packets = 0;
        xLastDebugTime = now_ms;
    }
  }
}

bool teensyGetStatus(void) {
    return status;
}

float teensyGetRollRateP() {
    return myserial_pid_out.roll_p;
}

float teensyGetRollRateI() {
    return myserial_pid_out.roll_i;
}

float teensyGetRollRateD() {
    return myserial_pid_out.roll_d;
}

float teensyGetPitchRateP() {
    return myserial_pid_out.pitch_p;
}

float teensyGetPitchRateI() {
    return myserial_pid_out.pitch_i;
}

float teensyGetPitchRateD() {
    return myserial_pid_out.pitch_d;
}

static const DeckDriver teensy_deck = {
  .vid = 0xBC,
  .pid = 0x29,
  .name = "teensy",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
  .requiredEstimator = StateEstimatorTypeKalman,

  .init = teensyInit,
  .test = teensyTest,
};

DECK_DRIVER(teensy_deck);

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(snn_pid)
/**
 * @brief SNN PID rollrate P
 */
LOG_ADD(LOG_FLOAT, rollrate_p, &myserial_pid_out.roll_p)
/**
 * @brief SNN PID rollrate I
 */
LOG_ADD(LOG_FLOAT, rollrate_i, &myserial_pid_out.roll_i)
/**
 * @brief SNN PID rollrate D
 */
LOG_ADD(LOG_FLOAT, rollrate_d, &myserial_pid_out.roll_d)
/**
 * @brief SNN PID pitchrate P
 */
LOG_ADD(LOG_FLOAT, pitchrate_p, &myserial_pid_out.pitch_p)
/**
 * @brief SNN PID pitchrate I
 */
LOG_ADD(LOG_FLOAT, pitchrate_i, &myserial_pid_out.pitch_i)
/**
 * @brief SNN PID pitchrate D
 */
LOG_ADD(LOG_FLOAT, pitchrate_d, &myserial_pid_out.pitch_d)
/**
 * @brief SNN PID pitchrate P
 */
LOG_ADD(LOG_FLOAT, yawrate_p, &myserial_pid_out.yaw_p)
/**
 * @brief SNN PID pitchrate I
 */
LOG_ADD(LOG_FLOAT, yawrate_i, &myserial_pid_out.yaw_i)
/**
 * @brief SNN PID pitchrate D
 */
LOG_ADD(LOG_FLOAT, yawrate_d, &myserial_pid_out.yaw_d)
/**
 * @brief SNN PID status
 */
LOG_ADD(LOG_UINT8, status, &status)
LOG_GROUP_STOP(snn_pid)

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTeensy, &isInit)

PARAM_GROUP_STOP(deck)