/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_twr_anchor.c: Uwb two way ranging anchor implementation */


#include <string.h>
#include "lpsTwrTag.h"
#include "log.h"
#include "physicalConstants.h"
#include "FreeRTOS.h"
#include "task.h"
#include "configblock.h"
// #include "estimator_kalman.h"
#include "estimator.h"
// #include "estimator_complementary.h"
#include "swarm_info.h"

#include "debug.h"


// #define ANTENNA_OFFSET 154.6 // In meter
#define basicAddr 0xbccf851300000000
// Define: id = last_number_of_address - 5
static uint8_t selfID;
static locoAddress_t selfAddress;
// static const uint64_t antennaDelay = (ANTENNA_OFFSET * 499.2e6 * 128) / 299792458.0; // In radio tick

// Config
static lpsTwrAlgoOptions_t defaultOptions = {
   .tagAddress = 0xbccf000000000008,
   .anchorAddress = {
     0xbccf000000000000
 #if LOCODECK_NR_OF_TWR_ANCHORS > 6
     0xbccf000000000006,
 #endif
 #if LOCODECK_NR_OF_TWR_ANCHORS > 7
     0xbccf000000000007,
 #endif
   },
   .antennaDelay = LOCODECK_ANTENNA_DELAY,
   .rangingFailedThreshold = 6,

   .combinedAnchorPositionOk = false,

 #ifdef LPS_TDMA_ENABLE
   .useTdma = true,
   .tdmaSlot = TDMA_SLOT,
 #endif

   // To set a static anchor position from startup, uncomment and modify the
   // following code:
 //   .anchorPosition = {
 //     {timestamp: 1, x: 0.99, y: 1.49, z: 1.80},
 //     {timestamp: 1, x: 0.99, y: 3.29, z: 1.80},
 //     {timestamp: 1, x: 4.67, y: 2.54, z: 1.80},
 //     {timestamp: 1, x: 0.59, y: 2.27, z: 0.20},
 //     {timestamp: 1, x: 4.70, y: 3.38, z: 0.20},
 //     {timestamp: 1, x: 4.70, y: 1.14, z: 0.20},
 //   },
 //
 //   .combinedAnchorPositionOk = true,
};

static lpsTwrAlgoOptions_t* options = &defaultOptions;

typedef struct
{
  uint16_t distance[LOCODECK_NR_OF_TWR_ANCHORS + 1];
  float x[LOCODECK_NR_OF_TWR_ANCHORS + 1];
  float y[LOCODECK_NR_OF_TWR_ANCHORS + 1];
  float gz[LOCODECK_NR_OF_TWR_ANCHORS + 1];
  float h[LOCODECK_NR_OF_TWR_ANCHORS + 1];
  bool refresh[LOCODECK_NR_OF_TWR_ANCHORS + 1];
  bool keep_flying;
  int failedRanging[LOCODECK_NR_OF_TWR_ANCHORS];
} swarmInfo_t;
static swarmInfo_t state;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static bool rangingOk;

// Communication logic between each UWB
static bool current_mode_trans;
static uint8_t current_receiveID;

#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
static bool checkTurn;
static uint32_t checkTurnTick = 0;
#endif

// Median filter for distance ranging (size=3)
typedef struct
{
  uint16_t distance_history[3];
  uint8_t index_inserting;
} median_data_t;
static median_data_t median_data[LOCODECK_NR_OF_TWR_ANCHORS + 1];

static uint16_t median_filter_3(uint16_t *data)
{
  uint16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2]))
  {
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if ((data[1] <= data[0]) && (data[1] <= data[2]))
  {
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else
  {
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (options->antennaDelay / 2);

  if (current_mode_trans)
  {
    switch (txPacket.payload[0]) {
    case LPS_TWR_POLL:
      poll_tx = departure;
      break;
    case LPS_TWR_FINAL:
      final_tx = departure;
      break;
    case LPS_TWR_REPORT + 1:
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
      if ((current_receiveID == 0) || (current_receiveID - 1 == selfID))
      {
        // current_receiveID = current_receiveID;
        current_mode_trans = false;
        dwIdle(dev);
        dwSetReceiveWaitTimeout(dev, 10000);
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
        checkTurn = true;
        checkTurnTick = xTaskGetTickCount();
      }
      else
      {
        current_receiveID = current_receiveID - 1;
      }
#endif
      break;
    }
  }
  else
  {
    switch (txPacket.payload[0])
    {
    case LPS_TWR_ANSWER:
      answer_tx = departure;
      break;
    case LPS_TWR_REPORT:
      break;
    }
  }
}

static void rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (rxPacket.destAddress != selfAddress) {
    if (current_mode_trans)
    {
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
      current_mode_trans = false;
#endif
      dwIdle(dev);
      dwSetReceiveWaitTimeout(dev, 10000);
    }
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if (current_mode_trans)
  {
    switch (rxPacket.payload[LPS_TWR_TYPE])
    {
    case LPS_TWR_ANSWER:

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    case LPS_TWR_REPORT:
    {
      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;


      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn / LOCODECK_TS_FREQ;
      uint16_t calcDist = (uint16_t)(1000 * SPEED_OF_LIGHT * tprop);
      if (calcDist != 0)
      {
        uint16_t medianDist = median_filter_3(median_data[current_receiveID].distance_history);
        if (ABS(medianDist - calcDist) > 500)
          state.distance[current_receiveID] = medianDist;
        else
          state.distance[current_receiveID] = calcDist;
        median_data[current_receiveID].index_inserting++;
        if (median_data[current_receiveID].index_inserting == 3)
          median_data[current_receiveID].index_inserting = 0;
        median_data[current_receiveID].distance_history[median_data[current_receiveID].index_inserting] = calcDist;
        rangingOk = true;
        state.x[current_receiveID] = report->selfX;
        state.y[current_receiveID] = report->selfY;
        state.gz[current_receiveID] = report->selfGz;
        state.h[current_receiveID] = report->selfh;
        if (current_receiveID == 0)
          state.keep_flying = report->keep_flying;
        state.refresh[current_receiveID] = true;

        // DEBUG_PRINT("Received reciprocal distance measurement from ID %d: %u mm from pos (%.2f, %.2f, %.2f)\n", current_receiveID, calcDist, (double)state.x[current_receiveID], (double)state.y[current_receiveID], (double)state.h[current_receiveID]);
        if (isAnchor == 0)
        {
          distanceMeasurement_t dist;
          dist.distance = (float)state.distance[current_receiveID] / 1000.0f;
          dist.x = state.x[current_receiveID];
          dist.y = state.y[current_receiveID];
          dist.z = state.h[current_receiveID];
          dist.anchorId = current_receiveID;
          dist.stdDev = 0.25; // Make this depend on type of other crazyflie
          estimatorEnqueueDistance(&dist);
        }
      }

      lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(txPacket.payload + 2);
      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT + 1;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
      report2->reciprocalDistance = calcDist;

      // first load data into local variable to prevent memory misalignment warning
      float selfX2 = report2->selfX;
      float selfY2 = report2->selfY;
      float selfGz2 = report2->selfGz;
      float selfh2 = report2->selfh;

      // Fetch latest self state to include in the report
      swarmInfoGet(&selfX2, &selfY2, &selfGz2, &selfh2);

      report2->selfX = selfX2;
      report2->selfY = selfY2;
      report2->selfGz = selfGz2;
      report2->selfh = selfh2;
      
      report2->keep_flying = state.keep_flying;
      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(lpsTwrTagReportPayload_t));
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    }
    }
  }
  else
  {
    switch (rxPacket.payload[LPS_TWR_TYPE])
    {
    case LPS_TWR_POLL:
    {
      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      poll_rx = arival;
      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    }
    case LPS_TWR_FINAL:
    {
      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(txPacket.payload + 2);
      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      final_rx = arival;
      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
      memcpy(&report->pollRx, &poll_rx, 5);
      memcpy(&report->answerTx, &answer_tx, 5);
      memcpy(&report->finalRx, &final_rx, 5);
      
      // first load data into local variable to prevent memory misalignment warning
      float selfX= report->selfX;
      float selfY = report->selfY;
      float selfGz = report->selfGz;
      float selfh = report->selfh;

      // Fetch latest self state to include in the report
      swarmInfoGet(&selfX, &selfY, &selfGz, &selfh);
      report->selfX = selfX;
      report->selfY = selfY;
      report->selfGz = selfGz;
      report->selfh = selfh;

      report->keep_flying = state.keep_flying;
      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(lpsTwrTagReportPayload_t));
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    }
    case (LPS_TWR_REPORT + 1):
    {
      lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(rxPacket.payload + 2);
      uint8_t rangingID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
      if ((report2->reciprocalDistance) != 0)
      {
        // received distance has large noise
        uint16_t calcDist = report2->reciprocalDistance;
        uint16_t medianDist = median_filter_3(median_data[rangingID].distance_history);
        if (ABS(medianDist - calcDist) > 500)
          state.distance[rangingID] = medianDist;
        else
          state.distance[rangingID] = calcDist;
        median_data[rangingID].index_inserting++;
        if (median_data[rangingID].index_inserting == 3)
          median_data[rangingID].index_inserting = 0;
        median_data[rangingID].distance_history[median_data[rangingID].index_inserting] = calcDist;
        state.x[rangingID] = report2->selfX;
        state.y[rangingID] = report2->selfY;
        state.gz[rangingID] = report2->selfGz;
        state.h[rangingID] = report2->selfh;
        if (rangingID == 0)
          state.keep_flying = report2->keep_flying;
        state.refresh[rangingID] = true;

        // DEBUG_PRINT("Received reciprocal distance measurement from ID %d: %u mm from pos (%.2f, %.2f, %.2f)\n", rangingID, report2->reciprocalDistance, (double)state.x[rangingID], (double)state.y[rangingID], (double)state.h[rangingID]);
        // Push distance measurement to estimator if the drone is not an anchor
        if (isAnchor == 0)
        {
          distanceMeasurement_t dist;
          dist.distance = (float)state.distance[rangingID] / 1000.0f;
          dist.x = state.x[rangingID];
          dist.y = state.y[rangingID];
          dist.z = state.h[rangingID];
          dist.anchorId = rangingID;
          dist.stdDev = 0.25; // Make this depend on type of other crazyflie
          estimatorEnqueueDistance(&dist);
        }
      }
      rangingOk = true;
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
      uint8_t fromID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
      if (selfID == fromID + 1 || selfID == 0)
      {
        current_mode_trans = true;
        dwIdle(dev);
        dwSetReceiveWaitTimeout(dev, 1000);
        if (selfID == LOCODECK_NR_OF_TWR_ANCHORS)
          current_receiveID = 0;
        else
          current_receiveID = LOCODECK_NR_OF_TWR_ANCHORS;
        if (selfID == 0)
          current_receiveID = LOCODECK_NR_OF_TWR_ANCHORS - 1; // immediate problem
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }
      else
      {
#endif
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
      }
#endif
      break;
    }
    }
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  // TODO: Nothing is done with failedRanging. This should be calculated
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
      checkTurn = false;
#endif
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
    case eventReceiveTimeout:
    case eventReceiveFailed:
      if (current_mode_trans == true)
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }
      else
      {
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
        if (xTaskGetTickCount() > checkTurnTick + 20) // > 20ms
        {
          if (checkTurn == true)
          {
            current_mode_trans = true;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = selfAddress;
            txPacket.destAddress = basicAddr + current_receiveID;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
            checkTurn = false;
            break;
          }
        }
#endif
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
      }
      break;
    default:
      configASSERT(false);
  }

  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  selfID = (uint8_t)(configblockGetRadioAddress() & 0xF);
  selfAddress = basicAddr + selfID;

  // Communication logic between each UWB
  if (selfID == 0)
  {
    current_receiveID = (LOCODECK_NR_OF_TWR_ANCHORS + 1) - 1;
    current_mode_trans = true;
    dwSetReceiveWaitTimeout(dev, 1000);
  }
  else
  {
    // current_receiveID = 0;
    current_mode_trans = false;
    dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);
  }

  for (int i = 0; i < (LOCODECK_NR_OF_TWR_ANCHORS + 1); i++)
  {
    median_data[i].index_inserting = 0;
    state.refresh[i] = false;
  }

  state.keep_flying = false;

  DEBUG_PRINT("twrtag initialized with ID: %d\n", selfID);
#if (LOCODECK_NR_OF_TWR_ANCHORS + 1) > 2
  checkTurn = false;
#endif
  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

void uwbTwrTagSetOptions(lpsTwrAlgoOptions_t* newOptions) {
  options = newOptions;
}

float lpsTwrTagGetDistance(const uint8_t anchorId) {
  return state.distance[anchorId];
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < LOCODECK_NR_OF_TWR_ANCHORS) {
    *position = options->anchorPosition[anchorId];
    return true;
  }

  return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++) {
    unorderedAnchorList[i] = i;
  }

  return LOCODECK_NR_OF_TWR_ANCHORS;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;

  for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++) {
    if (state.failedRanging[i] < options->rangingFailedThreshold) {
      unorderedAnchorList[count] = i;
      count++;
    }
  }

  return count;
}

bool twrGetSwarmInfo(int robNum, uint16_t *range, float *x, float *y, float *gyroZ, float *height)
{
  // DEBUG_PRINT("twrGetSwarmInfo called for robot %d\n", robNum);
  if (state.refresh[robNum] == true)
  {
    state.refresh[robNum] = false;
    *range = state.distance[robNum];
    *x = state.x[robNum];
    *y = state.y[robNum];
    *gyroZ = state.gz[robNum];
    *height = state.h[robNum];
    return (true);
  }
  else
  {
    return (false);
  }
}

bool command_share(int RobIDfromControl, bool keep_flying)
{
  if (RobIDfromControl == 0)
  {
    state.keep_flying = keep_flying;
    return keep_flying;
  }
  else
  {
    return state.keep_flying;
  }
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, distance0, &state.distance[0])
LOG_ADD(LOG_UINT16, distance1, &state.distance[1])
LOG_ADD(LOG_UINT16, distance2, &state.distance[2])
LOG_ADD(LOG_UINT16, distance3, &state.distance[3])
LOG_ADD(LOG_UINT16, distance4, &state.distance[4])
LOG_GROUP_STOP(ranging)