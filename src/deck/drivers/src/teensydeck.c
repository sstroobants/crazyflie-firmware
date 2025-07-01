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
#include "led.h"
#include "ledseq.h"

#include "cf_math.h"

static bool isInit = false;
bool communicationStatus = false;
bool status = true;


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


// LED variables
ledseqStep_t seq_noteensy_def[] = {
  { true, LEDSEQ_WAITMS(75)},
  {false, LEDSEQ_WAITMS(75)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_noteensy = {
  .sequence = seq_noteensy_def,
  .led = SYS_LED,
};

void startNoTeensyLedSequence(void) {
    ledseqStopBlocking(&seq_calibrated); // Stop the alive sequence
    ledseqStopBlocking(&seq_alive); // Stop the alive sequence
    vTaskDelay(M2T(5));
    ledseqRunBlocking(&seq_noteensy); // Run the noteensy sequence
}

void stopNoTeensyLedSequence(void) {
    ledseqStopBlocking(&seq_noteensy); // Stop the noteensy sequence
    ledseqRunBlocking(&seq_calibrated); // Run the alive sequence
    ledseqRunBlocking(&seq_alive); // Run the alive sequence
}

void serialParseMessageOut(void)
{
  //Copy received buffer to structure
  memmove(&myserial_control_out,&serial_cf_msg_buf_out[1],sizeof(struct serial_control_out)-1);
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

    stateEstimateRoll = logGetFloat(idStateEstimateRoll);
    stateEstimatePitch = logGetFloat(idStateEstimatePitch);

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
    myserial_control_in.roll = stateEstimateRoll;
    myserial_control_in.pitch = stateEstimatePitch;
    myserial_control_in.roll_t = controllerRoll;
    myserial_control_in.pitch_t = controllerPitch;
    myserial_control_in.yaw_t = controllerYawRate;
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
        if (communicationStatus) {
          DEBUG_PRINT("Did not receive message on time, trying to resend\n");
          // Run led sequency to notify that we haven't received messages
          // And also stop the alive/calibrated sequence
          startNoTeensyLedSequence();
          communicationStatus = false;
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
            if (!communicationStatus) {
                DEBUG_PRINT("Connection (re-)gained\n");
                // Restart the alive/calibrated led sequence
                stopNoTeensyLedSequence();
                communicationStatus = true;
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

  // Register the LED sequence
  ledseqRegisterSequence(&seq_noteensy);
  
  //  Add delay to ensure the deck is ready before starting
  //  Now when we start, the Teensy should have finished initializing
  //  And the default LED sequences should have started. 
  // vTaskDelay(M2T(3000)); // Delay for 1 seconds 
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

  TickType_t xLastForwardTime = 0;
  TickType_t xLastBottomTime = 0;

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

        // For both sensors, check timeout
        if (myserial_control_out.dist_forward_new) {
          xLastForwardTime = T2M(xTaskGetTickCount());
        }

        if (myserial_control_out.dist_bottom_new) {
          xLastBottomTime = T2M(xTaskGetTickCount());
        }

        if ((T2M(xTaskGetTickCount()) - xLastForwardTime > 500) || (T2M(xTaskGetTickCount()) - xLastBottomTime > 500)) {
          // If we have not received a forward or bottom message for more than 500ms, notify teensy failure
          if (status) {
            DEBUG_PRINT("Timeout: No forward or bottom message received for >500ms\n");
            startNoTeensyLedSequence();
            status = false; 
          }
            // Run led sequency to notify that we haven't received messages
        } else {
            // If we have received both messages, reset the noteensy sequence
            if (!status) {
              DEBUG_PRINT("Received messages from the sensors\n");
              stopNoTeensyLedSequence();
              status = true;
            }
        }

    } else {
        vTaskDelayUntil(&xLastWakeTime, F2T(100));
        sending = true;
    }
    // Printing the amount of received messages over the last seconds
    uint32_t now_ms = T2M(xTaskGetTickCount());
    if (now_ms - xLastDebugTime > 10000) {
        DEBUG_PRINT("received %i messages in the last 10 seconds, spent %i ms sending, %i receiving\n", serial_cf_received_packets, sending_outer, receiving_outer);
        // DEBUG_PRINT("Last received message: ll: %i, ml: %i, mr: %i, rr: %i \n", myserial_control_out.dist_ll_forward, myserial_control_out.dist_ml_forward, myserial_control_out.dist_mr_forward, myserial_control_out.dist_rr_forward);
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
LOG_ADD(LOG_UINT8, forward_new, &myserial_control_out.dist_forward_new)
LOG_ADD(LOG_UINT16, bottom_ll, &myserial_control_out.dist_ll_bottom)
LOG_ADD(LOG_UINT16, bottom_ml, &myserial_control_out.dist_ml_bottom)
LOG_ADD(LOG_UINT16, bottom_mr, &myserial_control_out.dist_mr_bottom)
LOG_ADD(LOG_UINT16, bottom_rr, &myserial_control_out.dist_rr_bottom)
LOG_ADD(LOG_UINT8, bottom_new, &myserial_control_out.dist_bottom_new)
LOG_GROUP_STOP(teensy)

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTeensy, &isInit)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit) // hack so we can use the range messages in the complementary filter

PARAM_GROUP_STOP(deck)