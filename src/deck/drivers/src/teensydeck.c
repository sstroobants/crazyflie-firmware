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
 * teensydeck.c: driver for communication with a Teensy
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

struct serial_control_in myserial_control_in;
struct serial_control_out myserial_control_out;
uint8_t serial_cf_msg_buf_out[ 2*sizeof(struct serial_control_out) ] = {0};
uint16_t serial_cf_buf_out_cnt = 0;
int serial_cf_received_packets = 0;

bool receiving;
bool sending;

int receiving_outer = 0;
int sending_outer = 0;

/////////////INTERNAL LOG VARIABLES
logVarId_t idThrust;
float thrust;
logVarId_t idGyroX, idGyroY, idGyroZ, idAccX, idAccY, idAccZ;
float gyroX, gyroY, gyroZ, accX, accY, accZ;
logVarId_t idControllerRoll, idControllerPitch;
logVarId_t idControllerRollRate, idControllerPitchRate, idControllerYawRate;
float controllerRoll, controllerPitch;
float  controllerRollRate, controllerPitchRate, controllerYawRate;
logVarId_t idStateEstimateRoll, idStateEstimatePitch;
float stateEstimateRoll, stateEstimatePitch;
logVarId_t idPidRateRollOutput, idPidRatePitchOutput, idPidRateYawOutput;
float pidRateRollOutput, pidRatePitchOutput, pidRateYawOutput;

void serialParseMessageOut(void)
{
  //Copy received buffer to structure
  memmove(&myserial_control_out,&serial_cf_msg_buf_out[1],sizeof(struct serial_control_out)-1);
//   DEBUG_PRINT("Correct message received and storing\n");
//   DEBUG_PRINT("Stored roll p, i, d is %i, %i, %i\n", myserial_control_in.roll_p, myserial_control_in.roll_i, myserial_control_in.roll_d);
}

void setControlInMessage(void) 
{
    // Get roll input values and put them in the message
    thrust = logGetFloat(idThrust);
    gyroX = logGetFloat(idGyroX);
    gyroY = logGetFloat(idGyroY);
    gyroZ = logGetFloat(idGyroZ);
    accX = logGetFloat(idAccX);
    accY = logGetFloat(idAccY);
    accZ = logGetFloat(idAccZ);

    controllerRoll = logGetFloat(idControllerRoll);
    controllerPitch = logGetFloat(idControllerPitch);
    controllerYawRate = logGetFloat(idControllerYawRate);

    myserial_control_in.thrust = thrust;
    myserial_control_in.roll_gyro = gyroX;
    myserial_control_in.pitch_gyro = gyroY;
    myserial_control_in.yaw_gyro = gyroZ;
    myserial_control_in.x_acc = accX;
    myserial_control_in.y_acc = accY;
    myserial_control_in.z_acc = accZ;
    myserial_control_in.roll = controllerRoll;
    myserial_control_in.pitch = controllerPitch;
    myserial_control_in.yaw = controllerYawRate;
}

// Read a control out message over uart
void uartReadControlOutMessage(void) 
{
    uint8_t serial_cf_byte_in;
    // uart2Getchar(&serial_cf_byte_in);
    // if (!uart1GetCharWithTimeout(&serial_cf_byte_in, 100)) {
    if (!uart1GetDataWithTimeout(&serial_cf_byte_in, 200)) {
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
    if (serial_cf_buf_out_cnt > sizeof(struct serial_control_out)  ) {
        serial_cf_buf_out_cnt = 0;
        uint8_t checksum_in_local = 0;
        for(uint16_t i = 1; i < sizeof(struct serial_control_out) ; i++){
            checksum_in_local += serial_cf_msg_buf_out[i];
        }
        if(checksum_in_local == serial_cf_msg_buf_out[sizeof(struct serial_control_out)]){
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

// Send a ControlIn message via uart to Teensy
void uartSendControlInMessage(void)
{
    //Calculate checksum for outbound packet: 
    uint8_t *buf_send = (uint8_t *)&myserial_control_in;
    myserial_control_in.checksum_in = 0;
    for(uint16_t i = 0; i < sizeof(struct serial_control_in) - 1; i++){
        myserial_control_in.checksum_in += buf_send [i];
    }
    uint8_t startByte = START_BYTE_SERIAL_CF;
    // uart1SendDataDmaBlocking(1, &startByte);
    uart1SendData(1, &startByte);
    // uart2SendDataDmaBlocking(sizeof(struct serial_control_in), buf_send);
    uart1SendData(sizeof(struct serial_control_in), buf_send);
    // uart2SendData(1, &startByte);
    // uart2SendData(sizeof(struct serial_control_in), buf_send);
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
  //   uart1Init(115200);
  
  
  // get the logVarIds that are used to get the state/target info
  idGyroX = logGetVarId("gyro", "x");
  idGyroY = logGetVarId("gyro", "y");
  idGyroZ = logGetVarId("gyro", "z");
  idAccX = logGetVarId("acc", "x");
  idAccY = logGetVarId("acc", "y");
  idAccZ = logGetVarId("acc", "z");
  idThrust = logGetVarId("controller", "actuatorThrust");
  idControllerRoll = logGetVarId("controller", "roll");
  idControllerPitch = logGetVarId("controller", "pitch");
  idControllerRollRate = logGetVarId("controller", "rollRate");
  idControllerPitchRate = logGetVarId("controller", "pitchRate");
  idControllerYawRate = logGetVarId("controller", "yawRate");
  idStateEstimateRoll = logGetVarId("stateEstimate", "roll");
  idStateEstimatePitch = logGetVarId("stateEstimate", "pitch");
  idPidRateRollOutput = logGetVarId("pid_rate", "roll_output");
  idPidRatePitchOutput = logGetVarId("pid_rate", "pitch_output");
  idPidRateYawOutput = logGetVarId("pid_rate", "yaw_output");
  
//   Add delay to ensure the deck is ready before starting NOT NECESSARY?
//   vTaskDelay(M2T(1000)); // Delay for 1 seconds (1000 ms)
  xTaskCreate(teensyTask, TEENSY_TASK_NAME, TEENSY_TASK_STACKSIZE, NULL, TEENSY_TASK_PRI, NULL);
  
  DEBUG_PRINT("FINISHED CREATING TASK\n");
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
        setControlInMessage();
        uint32_t after = T2M(xTaskGetTickCount());
        uartSendControlInMessage();
        uint32_t after2 = T2M(xTaskGetTickCount());
        sending_outer = sending_outer + (after2 - after);
    } else if (receiving) {
        uint32_t now_ms = T2M(xTaskGetTickCount());
        uartReadControlOutMessage();
        uint32_t after = T2M(xTaskGetTickCount());
        receiving_outer = receiving_outer + (after - now_ms);
    } else {
        vTaskDelayUntil(&xLastWakeTime, F2T(100));
        sending = true;
    }
    // Printing the amount of received messages over the last seconds
    uint32_t now_ms = T2M(xTaskGetTickCount());
    if (now_ms - xLastDebugTime > 10000) {
        DEBUG_PRINT("received %i messages in the last second, spent %i ms sending, %i receiving\n", serial_cf_received_packets, sending_outer, receiving_outer);
        DEBUG_PRINT("Last received message: ll: %i, ml: %i, mr: %i, rr: %i \n", myserial_control_out.dist_ll_forward, myserial_control_out.dist_ml_forward, myserial_control_out.dist_mr_forward, myserial_control_out.dist_rr_forward);
        serial_cf_received_packets = 0;
        sending_outer = 0;
        receiving_outer = 0;
        xLastDebugTime = now_ms;
    }
  }
}

bool teensyGetStatus(void) {
    return status;
}


static const DeckDriver teensy_deck = {
  .vid = 0xBC,
  .pid = 0x29,
  .name = "teensy",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
//   .requiredEstimator = StateEstimatorTypeKalman,
  .init = teensyInit,
  .test = teensyTest,
};

DECK_DRIVER(teensy_deck);

/**
 * Logging variables for the command and reference signals for the
 * attitude controller
 */
LOG_GROUP_START(teensy)
/**
 * @brief teensy control status
 */
LOG_ADD(LOG_UINT8, status, &status)
LOG_ADD(LOG_UINT16, forward_ll, &myserial_control_out.dist_ll_forward)
LOG_ADD(LOG_UINT16, forward_ml, &myserial_control_out.dist_ml_forward)
LOG_ADD(LOG_UINT16, forward_mr, &myserial_control_out.dist_mr_forward)
LOG_ADD(LOG_UINT16, forward_rr, &myserial_control_out.dist_rr_forward)
LOG_ADD(LOG_UINT16, bottom_ll, &myserial_control_out.dist_ll_bottom)
LOG_ADD(LOG_UINT16, bottom_ml, &myserial_control_out.dist_ml_bottom)
LOG_ADD(LOG_UINT16, bottom_mr, &myserial_control_out.dist_mr_bottom)
LOG_ADD(LOG_UINT16, bottom_rr, &myserial_control_out.dist_rr_bottom)

LOG_GROUP_STOP(teensy)

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTeensy, &isInit)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)

PARAM_GROUP_STOP(deck)