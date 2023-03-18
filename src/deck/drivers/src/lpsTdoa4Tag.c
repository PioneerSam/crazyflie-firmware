/*
    lpsTdoa4Tag.c

    Created on : Nov.,21,2020
        Author : Wenda Zhao
        Email  : wenda.zhao@robotics.utias.utoronto.ca
    Inter-drone ranging and communication via UWB. (based on TDOA3 protocol)
*/

#include <string.h>
#include <stdlib.h>               // [change] for rand() function 
#include "FreeRTOS.h"
#include "task.h"
#include "libdw1000.h"
#include "mac.h"
#include "lpsTdoa4Tag.h"
#include "log.h"
#include "debug.h"
#include "estimator_kalman.h"
#include "eventtrigger.h"        // add eventtrigger logging

// declare events
EVENTTRIGGER(interRange, float, inter_ranging) 
EVENTTRIGGER(rAgent, float, rAgent_vx, float, rAgent_vy, float, rAgent_yr, float, rAgent_h)      // remote agent
EVENTTRIGGER(lAgent, float, lAgent_vx, float, lAgent_vy, float, lAgent_yr, float, lAgent_h)      // local  agent

#define TDOA4_RECEIVE_TIMEOUT 10000
// Packet formats
#define PACKET_TYPE_TDOA4 0x60                      // [Change]
#define LPP_HEADER 0
#define LPP_TYPE (LPP_HEADER + 1)
#define LPP_PAYLOAD (LPP_HEADER + 2)
// Useful constants
static const uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

// [change]: global variable for agent id
int AGENT_ID = 11;

// Agent msg context
typedef struct {
    uint8_t id;
    bool isUsed;
    uint8_t seqNr;
    uint32_t rxTimeStamp;
    uint32_t txTimeStamp;
    uint16_t distance;
    uint32_t distanceUpdateTime;
    bool isDataGoodForTransmission;

    double clockCorrection;
    int clockCorrectionBucket;
} anchorContext_t;

// This context struct contains all the required global values of the algorithm
static struct ctx_s {
    int anchorId;
    // Information about latest transmitted packet
    uint8_t seqNr;
    uint32_t txTime; // In UWB clock ticks

    // Next transmit time in system clock ticks
    uint32_t nextTxTick;
    int averageTxDelay; // ms

    // List of ids to transmit in remote data section
    uint8_t remoteTxId[REMOTE_TX_MAX_COUNT];
    uint8_t remoteTxIdCount;

    // The list of anchors to transmit and store is updated at regular intervals
    uint32_t nextAnchorListUpdate;

    // Remote anchor data
    uint8_t anchorCtxLookup[ID_COUNT];
    anchorContext_t anchorCtx[ANCHOR_STORAGE_COUNT];
    uint8_t anchorRxCount[ID_COUNT];
} ctx;

//[change]
static bool rangingOk;

// static tdoaEngineState_t engineState;
typedef struct {
  uint8_t type;
  uint8_t seq;
  uint32_t txTimeStamp;
  uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
  uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

typedef struct {
  rangePacketHeader3_t header;
  uint8_t remoteAnchorData;
} __attribute__((packed)) rangePacket3_t;

// // lppShortAnchorPos_s is defined in locodeck.h, here we define a new msg for TDoA4 
// // [New] lpp packet (transmission data): limitation is 11 float num (TODO: validate)
// struct lppShortAgentInputs {
//     float rAgent_data[4];
// } __attribute__((packed));

// [New] lpp packet (transmission data): limitation is 11 float num (TODO: validate)
struct lppShortAgentInput_s {
    float agent_info[4];         // vx, vy, gz, h
} __attribute__((packed));

// // Data struct for remote agent info 
// static struct remoteAgentInfo_s{
//     uint8_t remoteAgentID;           // source Agent (where the msg is sent from)
//     int destAgentID;             // destination Agent (where the msg is sent to)
//     bool hasDistance;
//     struct lppShortAgentInput_s remoteData;
//     float ranging;
// }remoteAgentInfo;                

//[DEBUG]: log parameter
// static float log_range;          // distance 
static int   log_rAgentID;       // remote agent ID 

// [Sam] Define a state to contain the remote agent states (input) for relative localization
typedef struct{
  float_t inter_range;
  float_t remote_vx;
  float_t remote_vy;
  float_t remote_gz;
  float_t remote_h;

  uint8_t remoteAgentID;           // source Agent (where the msg is sent from)

  bool refresh;
  bool keep_flying;
}swarminfo_t;

// Initialize the state
static swarminfo_t ra_inputstate;

// Sam temporary variables
// static float_t sam_temp;

/* ---------------------------- help functions ---------------------------- */
static anchorContext_t* getContext(uint8_t anchorId) {
  uint8_t slot = ctx.anchorCtxLookup[anchorId];
  if (slot == ID_WITHOUT_CONTEXT) {
    return 0;
  }

  return &ctx.anchorCtx[slot];
}

static void clearAnchorRxCount() {
  memset(&ctx.anchorRxCount, 0, ID_COUNT);
}

static void removeAnchorContextsNotInList(const uint8_t* id, const uint8_t count) {
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    anchorContext_t* anchorCtx = &ctx.anchorCtx[i];
    if (anchorCtx->isUsed) {
      const uint8_t ctxId = anchorCtx->id;
      bool found = false;
      for (int j = 0; j < count; j++) {
        if (id[j] == ctxId) {
          found = true;
          break;
        }
      }

      if (!found) {
        ctx.anchorCtxLookup[ctxId] = ID_WITHOUT_CONTEXT;
        anchorCtx->isUsed = false;
      }
    }
  }
}

static void createAnchorContext(const uint8_t id) {
  if (ctx.anchorCtxLookup[id] != ID_WITHOUT_CONTEXT) {
    // Already has a context, we're done
    return;
  }

  for (uint8_t i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    anchorContext_t* anchorCtx = &ctx.anchorCtx[i];
    if (!anchorCtx->isUsed) {
      ctx.anchorCtxLookup[id] = i;

      memset(anchorCtx, 0, sizeof(anchorContext_t));
      anchorCtx->id = id;
      anchorCtx->isUsed = true;

      break;
    }
  }
}

static void createAnchorContextsInList(const uint8_t* id, const uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    createAnchorContext(id[i]);
  }
}

static void purgeData() { // clear the data
  uint32_t now = xTaskGetTickCount();
  uint32_t acceptedCreationTime = now - DISTANCE_VALIDITY_PERIOD;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    anchorContext_t* anchorCtx = &ctx.anchorCtx[i];
    if (anchorCtx->isUsed) {
      if (anchorCtx->distanceUpdateTime < acceptedCreationTime) {
        anchorCtx->distance = 0;

        anchorCtx->clockCorrection = 0.0;
        anchorCtx->clockCorrectionBucket = 0;
      }
    }
  }
}

// This function is called at regular intervals to update lists containing data
// about which anchors to store and add to outgoing messages. This
// update might take some time but this should not be a problem since the TX
// times are randomized anyway. The intention is that we could plug in clever
// algorithms here that optimizes which anchors to use.
static void updateAnchorLists() {
  // Randomize which anchors to use

  static uint8_t availableId[ID_COUNT];
  static bool availableUsed[ID_COUNT];
  memset(availableId, 0, sizeof(availableId));
  memset(availableUsed, 0, sizeof(availableUsed));
  int availableCount = 0;

  static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
  memset(ctxts, 0, sizeof(ctxts));

  // Collect all anchors we have got a message from
  for (int i = 0; i < ID_COUNT; i++) {
    if (ctx.anchorRxCount[i] != 0) {
      availableId[availableCount++] = i;
    }
  }

  // Out of all anchors that we have received messages from, pick two
  // randomized subsets for storage and TX ids
  uint8_t remoteTXIdIndex = 0;
  uint8_t contextIndex = 0;
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    int start = rand() % availableCount;
    // Scan forward until we find an anchor
    for (int j = start; j < (start + availableCount); j++) {
      const int index = j % availableCount;
      if (!availableUsed[index]) {

        const int id = availableId[index];
        if (remoteTXIdIndex < REMOTE_TX_MAX_COUNT) {
          ctx.remoteTxId[remoteTXIdIndex++] = id;
        }
        if (contextIndex < ANCHOR_STORAGE_COUNT) {
          ctxts[contextIndex++] = id;
        }

        availableUsed[index] = true;
        break;
      }
    }
  }

  removeAnchorContextsNotInList(ctxts, contextIndex);
  createAnchorContextsInList(ctxts, contextIndex);

  ctx.remoteTxIdCount = remoteTXIdIndex;

  clearAnchorRxCount();

  // Set the TX rate based on the number of transmitting anchors around us
  // Aim for 400 messages/s. Up to 8 anchors: 50 Hz / anchor
  
  

  float freq = SYSTEM_TX_FREQ / (availableCount + 1);
  if (freq > (float) ANCHOR_MAX_TX_FREQ) {  //[change]: add (float)
    freq = ANCHOR_MAX_TX_FREQ;
  }
  if (freq < (float) ANCHOR_MIN_TX_FREQ) { //[change]: add (float)
    freq = ANCHOR_MIN_TX_FREQ;
  }

  ctx.averageTxDelay = 1000.0f / freq;

  purgeData();
}

/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 and round up */
static void adjustTxRxTime(dwTime_t *time)
{
  time->full = (time->full & ~((1 << 9) - 1)) + (1 << 9);
}

static dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev)
{
  dwTime_t transmitTime = { .full = 0 };
  dwGetSystemTimestamp(dev, &transmitTime);

  // Add guard and preamble time
  transmitTime.full += TDMA_GUARD_LENGTH;
  transmitTime.full += PREAMBLE_LENGTH;

  // And some extra
  transmitTime.full += TDMA_EXTRA_LENGTH;

  // TODO krri Adding randomization on this level adds a long delay, is it worth it?
  // The randomization on OS level is quantized to 1 ms (tick time of the system)
  // Add a high res random to smooth it out
  // uint32_t r = rand();
  // uint32_t delay = r % TDMA_HIGH_RES_RAND;
  // transmitTime.full += delay;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);

  return transmitTime;
}

static double calculateClockCorrection(anchorContext_t* anchorCtx, int remoteTxSeqNr, uint32_t remoteTx, uint32_t rx)
{
  double result = 0.0d;

  // Assigning to uint32_t truncates the diffs and takes care of wrapping clocks
  uint32_t tickCountRemote = remoteTx - anchorCtx->txTimeStamp;
  uint32_t tickCountLocal = rx - anchorCtx->rxTimeStamp;

  if (tickCountRemote != 0) {
    result = (double)tickCountLocal / (double)tickCountRemote;
  }

  return result;
}

static bool extractFromPacket(const rangePacket3_t* rangePacket, uint32_t* remoteRx, uint8_t* remoteRxSeqNr) {
  const void* anchorDataPtr = &rangePacket->remoteAnchorData;
  // loop over all the remote agents' info
    for (uint8_t i = 0; i < rangePacket->header.remoteCount; i++) {
        remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;
        // if the radio packet is sending to this agent --> to twr
        const uint8_t id = anchorData->id;
        if (id == ctx.anchorId) {
        *remoteRxSeqNr = anchorData->seq & 0x7f;
        *remoteRx = anchorData->rxTimeStamp;
        return true;
        }
        // else --> move the pointer away from the distance msg
        // currently the other agents distance msg is not used 
        bool hasDistance = ((anchorData->seq & 0x80) != 0);
        if (hasDistance) {
        anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        } else {
        anchorDataPtr += sizeof(remoteAnchorDataShort_t);
        }
    }
    return false;
}

static void fillClockCorrectionBucket(anchorContext_t* anchorCtx) {
    if (anchorCtx->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
      anchorCtx->clockCorrectionBucket++;
    }
}

static bool emptyClockCorrectionBucket(anchorContext_t* anchorCtx) {
    if (anchorCtx->clockCorrectionBucket > 0) {
      anchorCtx->clockCorrectionBucket--;
      return false;
    }

    return true;
}

static bool updateClockCorrection(anchorContext_t* anchorCtx, double clockCorrection) {
  const double diff = clockCorrection - anchorCtx->clockCorrection;
  bool sampleIsAccepted = false;

  if (-CLOCK_CORRECTION_ACCEPTED_NOISE < diff && diff < CLOCK_CORRECTION_ACCEPTED_NOISE) {
    // LP filter
    anchorCtx->clockCorrection = anchorCtx->clockCorrection * (1.0d - CLOCK_CORRECTION_FILTER) + clockCorrection * CLOCK_CORRECTION_FILTER;

    fillClockCorrectionBucket(anchorCtx);
    sampleIsAccepted = true;
  } else {
    if (emptyClockCorrectionBucket(anchorCtx)) {
      if (CLOCK_CORRECTION_SPEC_MIN < clockCorrection && clockCorrection < CLOCK_CORRECTION_SPEC_MAX) {
        anchorCtx->clockCorrection = clockCorrection;
      }
    }
  }

  return sampleIsAccepted;
}



/* ------------------------------ Primary functions ------------------------------- */
// [important] Computing TWR 
static uint16_t calculateDistance(anchorContext_t* anchorCtx, int remoteRxSeqNr, uint32_t remoteTx, uint32_t remoteRx, uint32_t rx)
{
  // Check that the remote received seq nr is our latest tx seq nr
  if (remoteRxSeqNr == ctx.seqNr && anchorCtx->clockCorrection > 0.0d) {
    uint32_t localTime = rx - ctx.txTime;
    uint32_t remoteTime = (uint32_t)((double)(remoteTx - remoteRx) * anchorCtx->clockCorrection);
    uint32_t distance = (localTime - remoteTime) / 2;

    return distance & 0xfffful;
  } else {
    return 0;
  }
}

// [New]: get the LPP transmitted data, function #2
static void handleLppShortPacket(const uint8_t *data, const int length) {
  uint8_t type = data[0];
  if (type == LPP_SHORT_AGENT_INFO) {
    struct lppShortAgentInput_s *rData = (struct lppShortAgentInput_s*)&data[1];
        // save and use the remote agent pos data  
        // [Sam] Commented               
        // remoteAgentInfo.remoteData.rAgent_data[0] = rData->rAgent_data[0];     
        // remoteAgentInfo.remoteData.rAgent_data[1] = rData->rAgent_data[1];     
        // remoteAgentInfo.remoteData.rAgent_data[2] = rData->rAgent_data[2];     
        // remoteAgentInfo.remoteData.rAgent_data[3] = rData->rAgent_data[3]; 

        // [Sam] Saved the recieved package
        ra_inputstate.remote_vx = rData->agent_info[0];
        ra_inputstate.remote_vy = rData->agent_info[1];
        ra_inputstate.remote_gz = rData->agent_info[2];
        ra_inputstate.remote_h = rData->agent_info[3];
    }
    else{
        // no remote agent pos data
        // remoteAgentInfo.remoteData.rAgent_data[0] = 255;     
        // remoteAgentInfo.remoteData.rAgent_data[1] = 255;     
        // remoteAgentInfo.remoteData.rAgent_data[2] = 255; 
        // remoteAgentInfo.remoteData.rAgent_data[3] = 255; 

        ra_inputstate.remote_vx = 255;
        ra_inputstate.remote_vy = 255;
        ra_inputstate.remote_gz = 255;
        ra_inputstate.remote_h = 255;
    }
}

// [New]: get the LPP transmitted data, function #1
static void handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket) {
    const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
    const int32_t startOfLppDataInPayload = rangePacketLength;
    const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
    const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;
    //   printf("payloadLentgh is %d\r\n",(int)payloadLength);
    //   printf("startOfLppDataInPayload is %d\r\n",(int)startOfLppDataInPayload);
    if (lppDataLength > 0) {
        const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
        if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
            const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
            handleLppShortPacket(&rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
      }
    }
}

// [New]: Update the remote agent info, also get the rangeDataLength --> for LPP packet
// [Note]: store the remote agent info. for tdoa computation
// Currently, this function is only used to extract the appended LPP msg
static int updateRemoteAgentData(const void* payload){
    const rangePacket3_t* packet = (rangePacket3_t*)payload;
    const void* anchorDataPtr = &packet->remoteAnchorData;
    // loop over all remote agent packet info, should save the remote agent data
    for(uint8_t i = 0; i<packet->header.remoteCount; i++){
        remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;
        /* comment out unused value*/
        // uint8_t remoteId = anchorData->id;
        // int64_t remoteRxTime = anchorData->rxTimeStamp;
        // uint8_t remoteSeqNr = anchorData->seq & 0x7f;

        /* ----------------- store the remote agent info ----------------- */
        // if (isValidTimeStamp(remoteRxTime)) {
        //    tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
        // }
        /* ----------------- --------------------------- ----------------- */
        bool hasDistance = ((anchorData->seq & 0x80) != 0);
        if (hasDistance) {
        /* comment out unused value*/
        // int64_t tof = anchorData->distance;

        /* ----------------- store the remote agent info ----------------- */
        // if (isValidTimeStamp(tof)) {
        //     tdoaStorageSetTimeOfFlight(anchorCtx, remoteId, tof);

        //     uint8_t anchorId = tdoaStorageGetId(anchorCtx);
        //     tdoaStats_t* stats = &engineState.stats;
        //     if (anchorId == stats->anchorId && remoteId == stats->remote_id) {
        //     stats->tof = (uint16_t)tof;
        //     }
        // }
        /* ----------------- --------------------------- ----------------- */
            anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        } else {
            anchorDataPtr += sizeof(remoteAnchorDataShort_t);
        }
    }
    return (uint8_t*)anchorDataPtr - (uint8_t*)packet;
}

// get range data from message
static void handleRangePacket(const uint32_t rxTime, const packet_t* rxPacket, const int dataLength)
{
  //[change] packet code is slightly different 
  //     in CF: locoAddress_t sourceAddress =>  uint64_t sourceAddress
  // in anchor: uint8_t sourceAddress[8]
  // similar to destAddress
  const uint8_t remote_id = rxPacket->sourceAddress;             // remote Agent ID, where the packet was sent from.
  log_rAgentID = remote_id;                                      // [LOG] log the remote Agent ID 

  // remoteAgentInfo.remoteAgentID = remote_id;                     // save to remoteAgent data structure
  ra_inputstate.remoteAgentID = remote_id;

  ctx.anchorRxCount[remote_id]++;
  anchorContext_t* anchorCtx = getContext(remote_id);            // get the msg from the packet
  if (anchorCtx) {
    const rangePacket3_t* rangePacket = (rangePacket3_t *)rxPacket->payload;

    uint32_t remoteTx = rangePacket->header.txTimeStamp;
    uint8_t remoteTxSeqNr = rangePacket->header.seq;

    double clockCorrection = calculateClockCorrection(anchorCtx, remoteTxSeqNr, remoteTx, rxTime);
    if (updateClockCorrection(anchorCtx, clockCorrection)) {
        anchorCtx->isDataGoodForTransmission = true;
        uint32_t remoteRx = 0;
        uint8_t remoteRxSeqNr = 0;
        bool dataFound = extractFromPacket(rangePacket, &remoteRx, &remoteRxSeqNr);
        if (dataFound) {
            //[important]: here is the range distance data!
            uint16_t distance = calculateDistance(anchorCtx, remoteRxSeqNr, remoteTx, remoteRx, rxTime);
            if (distance > MIN_TOF) {
                // The distance here is the tick count, need to time M_PER_TICK to compute range [m]
                anchorCtx->distance = distance;     
                anchorCtx->distanceUpdateTime = xTaskGetTickCount();
                // meter per tick
                float M_PER_TICK = 0.0046917639786157855;
                // [LOG] log the ranging distance

                // [Sam] Commented NOT USED
                // remoteAgentInfo.ranging = (float) distance * M_PER_TICK - (float)ANTENNA_OFFSET_INTER;   // save the range in meters. 
                // log_range = remoteAgentInfo.ranging;

                // [Sam] Once we have the ranging info, we can save it in our listener state
                ra_inputstate.inter_range = (float) distance * M_PER_TICK - (float)ANTENNA_OFFSET_INTER;

                // only extract remote agent pos and event logging when we have valid ranging info.
                // [important] get transmitted info. (dummy data for now)  (changed from lpsTdoa3Tag.c (rxcallback))
                // since we have extracted the ranging info., here we only extract the LPP packets
                int rangeDataLength = updateRemoteAgentData(rangePacket);   // get the length of LPP packets 
                handleLppPacket(dataLength, rangeDataLength, rxPacket);

                // // After (1) get remoteAgent ID, (2) compute the ranging, and (3) get the remoteAgent pos

                // interRange event
                // eventTrigger_interRange_payload.remote_id = ra_inputstate.remoteAgentID;
                eventTrigger_interRange_payload.inter_ranging = ra_inputstate.inter_range;
                
                // // rAgent event [Wenda]
                // eventTrigger_rAgent_payload.rAgent_vx = remoteAgentInfo.remoteData.rAgent_data[0];
                // eventTrigger_rAgent_payload.rAgent_vy = remoteAgentInfo.remoteData.rAgent_data[1];
                // eventTrigger_rAgent_payload.rAgent_yr = remoteAgentInfo.remoteData.rAgent_data[2];
                // eventTrigger_rAgent_payload.rAgent_h  = remoteAgentInfo.remoteData.rAgent_data[3];

                // rAgent event [Sam]
                eventTrigger_rAgent_payload.rAgent_vx = ra_inputstate.remote_vx;
                eventTrigger_rAgent_payload.rAgent_vy = ra_inputstate.remote_vy;
                eventTrigger_rAgent_payload.rAgent_yr = ra_inputstate.remote_gz;
                eventTrigger_rAgent_payload.rAgent_h  = ra_inputstate.remote_h;
                
                // lAgent event
                float local_data[4];  
                // call the function to get current local data
                // [Sam] Changed for local agent obtained from kalman filter
                estimatorKalmanGetSwarmInfo(&local_data[0], &local_data[1], &local_data[2], &local_data[3]);
                eventTrigger_lAgent_payload.lAgent_vx = local_data[0];
                eventTrigger_lAgent_payload.lAgent_vy = local_data[1];
                eventTrigger_lAgent_payload.lAgent_yr = local_data[2];
                eventTrigger_lAgent_payload.lAgent_h = local_data[3];

                // call the event logging on those three events: Interrange + remote agent event + local agent event
                eventTrigger(&eventTrigger_interRange);
                eventTrigger(&eventTrigger_rAgent);
                eventTrigger(&eventTrigger_lAgent);
            }
        }
    } else {
        anchorCtx->isDataGoodForTransmission = false;
    }
    // [change] follow uwb_tdoa_anchor3.c in the anchor code
    rangingOk = anchorCtx->isDataGoodForTransmission;
    anchorCtx->rxTimeStamp = rxTime;
    anchorCtx->seqNr = remoteTxSeqNr;
    anchorCtx->txTimeStamp = remoteTx;
  }
}

//[New]: moved from lpp.c in anchor code
static void lppHandleShortPacket(uint8_t *data, size_t length)
{
    // consider how to use the LPP msg
    if (length < 1) return;
    int type  = data[0];

    switch(type) {
        case LPP_SHORT_ANCHOR_POSITION:
        {
            // not used now. do nothing
            break;
        }
        case LPP_SHORT_REBOOT:
        { // not used now. do nothing
            break;
        }
        case LPP_SHORT_MODE:
        { // used to switch Agent mode
            // struct lppShortMode_s* modeInfo = (struct lppShortMode_s*)&data[1];
            DEBUG_PRINT("Receive LPP_SHORT_MODE !!!!!!!!!!! \n");
            // Set new mode
            // if (modeInfo->mode == LPP_SHORT_MODE_TWR) {
            //     MODE = lpsMode_TWR;
            // } else if (modeInfo->mode == LPP_SHORT_MODE_TDOA2) {
            //     MODE = lpsMode_TDoA2;
            // } else if (modeInfo->mode == LPP_SHORT_MODE_TDOA3) {
            //     // DEBUG_PRINT("Set mode to be tdoa3!!!!! \n");
            //     MODE = lpsMode_TDoA3;
            // }else if (modeInfo->mode == LPP_SHORT_MODE_TDOA4) {
            //     MODE = lpsMode_TDoA4;
            // }
            break;
        }
        case LPP_SHORT_UWB:
        { // not used for now. Do nothing
            break;
        }
        case LPP_SHORT_UWB_MODE:
        { // not used for now. Do nothing
            break;
        }
    }
}

// Main function after receive an uwb message
static void handleRxPacket(dwDevice_t *dev)
{
    //   int xEnd=0; int xDifference=0;
    static packet_t rxPacket;
    dwTime_t rxTime = { .full = 0 };

    dwGetRawReceiveTimestamp(dev, &rxTime);
    dwCorrectTimestamp(dev, &rxTime);

    int dataLength = dwGetDataLength(dev);
    rxPacket.payload[0] = 0;
    dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

    if (dataLength == 0) {
        return;
    }
    // DEBUG_PRINT("Receive radio packet \n");
    switch(rxPacket.payload[0]) {                                 // check the header 
    case PACKET_TYPE_TDOA4:       
        handleRangePacket(rxTime.low32, &rxPacket, dataLength);   // get range and agent ID
        break;
    case SHORT_LPP:                                               // handle SHORT_LPP msg
        // [Note] Need the (int) in front of rxPacket.destAddress
        // DEBUG_PRINT("Receive Short LPP\n");
        // DEBUG_PRINT("rxPacket.destAddress is %d \n",(int)rxPacket.destAddress);
        // DEBUG_PRINT("ctx.anchorId is %d \n",(int)ctx.anchorId);
        if ((int)rxPacket.destAddress == ctx.anchorId) {          // the lpp is sent to this Agent. 
            // DEBUG_PRINT("Receive the correct Short LPP packet !!!!!!!!!!!!\n");
            // testing switch time//
            // xStart_s = T2M(xTaskGetTickCount());
            lppHandleShortPacket(&rxPacket.payload[1], dataLength - MAC802154_HEADER_LENGTH - 1);
        }
        break;
    default:
        // Do nothing
        break;
    }
}


/* ---- low-level set-upc odes to receive and transmit UWB data----*/
static void setupRx(dwDevice_t *dev)
{
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static int populateTxData(rangePacket3_t *rangePacket)
{
  // rangePacket->header.type already populated
  rangePacket->header.seq = ctx.seqNr;
  rangePacket->header.txTimeStamp = ctx.txTime;

  uint8_t remoteAnchorCount = 0;
  uint8_t* anchorDataPtr = &rangePacket->remoteAnchorData;
  for (uint8_t i = 0; i < ctx.remoteTxIdCount; i++) {
    remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*) anchorDataPtr;

    uint8_t id = ctx.remoteTxId[i];
    anchorContext_t* anchorCtx = getContext(id);

    if (anchorCtx->isDataGoodForTransmission) {
      anchorData->id = id;
      anchorData->seq = anchorCtx->seqNr;
      anchorData->rxTimeStamp = anchorCtx->rxTimeStamp;

      if (anchorCtx->distance > 0) {
        anchorData->distance = anchorCtx->distance;
        anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        anchorData->seq |= 0x80;
      } else {
        anchorDataPtr += sizeof(remoteAnchorDataShort_t);
      }

      remoteAnchorCount++;
    }
  }
  rangePacket->header.remoteCount = remoteAnchorCount;

  return (uint8_t*)anchorDataPtr - (uint8_t*)rangePacket;
}

// Set TX data in the radio TX buffer to send: sourceAddress, destAddress, LPP.position
// [important]: transmit the UWB packet 
static void setTxData(dwDevice_t *dev)
{
  static packet_t txPacket;
  static bool firstEntry = true;
  static int lppLength = 0;

  if (firstEntry) {
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    // [change]: add '&' in front of txPacket 
    memcpy(&txPacket.sourceAddress, base_address, 8);
    txPacket.sourceAddress = ctx.anchorId;
    memcpy(&txPacket.destAddress, base_address, 8);
    txPacket.destAddress = 0xff;
    txPacket.payload[0] = PACKET_TYPE_TDOA4;

    firstEntry = false;
  }
    int rangePacketSize = populateTxData((rangePacket3_t *)txPacket.payload);
    // LPP anchor position is currently sent in all packets
    txPacket.payload[rangePacketSize + LPP_HEADER] = SHORT_LPP;
    txPacket.payload[rangePacketSize + LPP_TYPE] = LPP_SHORT_AGENT_INFO;   // [Change] define a new type for tdoa4 inter-drone msg

    struct lppShortAgentInput_s *rData = (struct lppShortAgentInput_s*) &txPacket.payload[rangePacketSize + LPP_PAYLOAD];
    /*------ Send the info. of interest--------*/
    // For more agents and anchors, LPP packet size will be limited
    // float dummy_pos[3] = {(float)AGENT_ID+(float)0.15, (float)AGENT_ID+(float)0.25, (float)AGENT_ID+(float)0.35};
    // memcpy(pos->rAgent_data, dummy_data, 4 * sizeof(float));

    /* share current position state from EKF using UWB*/
    // consider a better data structure design
    float shared_data[4];  
    // [Sam] Changed get local information
    estimatorKalmanGetSwarmInfo(&shared_data[0], &shared_data[1], &shared_data[2], &shared_data[3]);
    // send the local input to the other agents
    
    // // for testing
    // shared_data[0] = AGENT_ID + 0.1;
    // shared_data[1] = AGENT_ID + 0.2;
    // shared_data[2] = AGENT_ID + 0.3;
    // shared_data[3] = AGENT_ID + 0.4;

    memcpy(rData->agent_info, shared_data, 4 * sizeof(float));
    lppLength = 2 + sizeof(struct lppShortAgentInput_s);

    dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + rangePacketSize + lppLength);
}

static void setupTx(dwDevice_t *dev)
{
    dwTime_t txTime = findTransmitTimeAsSoonAsPossible(dev);
    ctx.txTime = txTime.low32;
    ctx.seqNr = (ctx.seqNr + 1) & 0x7f;

    setTxData(dev);

    dwNewTransmit(dev);
    dwSetDefaults(dev);
    dwSetTxRxTime(dev, txTime);

    dwStartTransmit(dev);
}

static uint32_t randomizeDelayToNextTx()
{
    const uint32_t interval = 10;

    uint32_t r = rand();
    uint32_t delay = ctx.averageTxDelay + r % interval - interval / 2;

    return delay;
}

static uint32_t startNextEvent(dwDevice_t *dev, uint32_t now)
{
    dwIdle(dev);

    if (ctx.nextTxTick < now) {
        uint32_t newDelay = randomizeDelayToNextTx();
        ctx.nextTxTick = now + M2T(newDelay);

        setupTx(dev);
    } else {
        setupRx(dev);
    }

    return ctx.nextTxTick - now;
}

//// [change]: not used now, comment out
// static void sendTdoaToEstimatorCallback(tdoaMeasurement_t* tdoaMeasurement) {
//   estimatorKalmanEnqueueTDOA(tdoaMeasurement);
// }

// [Sam] Adding the relative localization to get the remote agent state (listener)
bool getRemoteInfo(float* dij, float* vxj, float* vyj, float* rj, float* hj){
  // we can obtained the information from what we have recieved and stored in ra_inputstate
  *dij = ra_inputstate.inter_range;
  *vxj = ra_inputstate.remote_vx;
  *vyj = ra_inputstate.remote_vy;
  *rj = ra_inputstate.remote_gz;
  *hj = ra_inputstate.remote_h;
  return true;
}

//------------------------- Init and Event ----------------------------------//
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
// Initialize
static void tdoa4Init(dwDevice_t *dev)
{
    dwSetReceiveWaitTimeout(dev, TDOA4_RECEIVE_TIMEOUT);
    dwCommitConfiguration(dev);

    rangingOk = false;
    // manually set the Agent ID
    ctx.anchorId = AGENT_ID;   
    ctx.seqNr = 0;
    ctx.txTime = 0;
    ctx.nextTxTick = 0;
    ctx.averageTxDelay = 1000.0 / ANCHOR_MAX_TX_FREQ;
    ctx.remoteTxIdCount = 0;
    ctx.nextAnchorListUpdate = 0;

    memset(&ctx.anchorCtxLookup, ID_WITHOUT_CONTEXT, ID_COUNT);
    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
        ctx.anchorCtx[i].isUsed = false;
    }
    clearAnchorRxCount();
    srand(ctx.anchorId);
}

// TDOA4 Event
static uint32_t tdoa4UwbEvent(dwDevice_t *dev, uwbEvent_t event)
{
    //   int xStart=0; int xEnd=0; int xDifference=0;
    switch (event) {
        case eventPacketReceived: {
            handleRxPacket(dev);
        }
        break;
        default:
        // Nothing here
        break;
    }

    uint32_t now = xTaskGetTickCount();
    if (now > ctx.nextAnchorListUpdate) {
        updateAnchorLists();
        ctx.nextAnchorListUpdate = now + ANCHOR_LIST_UPDATE_INTERVAL;
    }

    uint32_t timeout_ms = startNextEvent(dev, now);
    return timeout_ms;
}

//-----------------Move the algorithm from lpstdoa3--------------------//
static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {

    // a walk around. 

    return true;
}

// [Note]: How are these two functions called??
// [Change]: Move the updateAnchorLists() function from anchor code
static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    // get the anchor id num that I received msg from.
    // do not need to use the inputs: uint8_t unorderedAnchorList[], const int maxListSize
    static uint8_t availableId[ID_COUNT];
    static bool availableUsed[ID_COUNT];
    memset(availableId, 0, sizeof(availableId));
    memset(availableUsed, 0, sizeof(availableUsed));
    int availableCount = 0;

    static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
    memset(ctxts, 0, sizeof(ctxts));

    // Collect all anchors we have got a message from
    for (int i = 0; i < ID_COUNT; i++) {
        if (ctx.anchorRxCount[i] != 0) {
        availableId[availableCount++] = i;
        }
    }

    return availableCount;
}

// [Change]: Move the updateAnchorLists() function from anchor code
static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    // get the anchor id num that I received msg from.
    static uint8_t availableId[ID_COUNT];
    static bool availableUsed[ID_COUNT];
    memset(availableId, 0, sizeof(availableId));
    memset(availableUsed, 0, sizeof(availableUsed));
    int availableCount = 0;

    static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
    memset(ctxts, 0, sizeof(ctxts));

    // Collect all anchors we have got a message from
    for (int i = 0; i < ID_COUNT; i++) {
        if (ctx.anchorRxCount[i] != 0) {
        availableId[availableCount++] = i;
        }
    }
    
    uint8_t remoteTXIdIndex = 0;
    uint8_t contextIndex = 0;
    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
        int start = rand() % availableCount;
        // Scan forward until we find an anchor
        for (int j = start; j < (start + availableCount); j++) {
            const int index = j % availableCount;
            if (!availableUsed[index]) {
                const int id = availableId[index];
                if (remoteTXIdIndex < REMOTE_TX_MAX_COUNT) {
                    ctx.remoteTxId[remoteTXIdIndex++] = id;
                    }
                if (contextIndex < ANCHOR_STORAGE_COUNT) {
                ctxts[contextIndex++] = id;
                }
                availableUsed[index] = true;
                break;
            }
        }
    }
   return contextIndex;
}


uwbAlgorithm_t uwbTdoa4TagAlgorithm = { // [change]: the name changed
    .init = tdoa4Init,   // the config is changed in init func
    .onEvent = tdoa4UwbEvent,
    // [change]: The following are needed (used for the GUI)
    // we don't need to modify them (let Bitcraze do that)
    .isRangingOk = isRangingOk,
    .getAnchorPosition = getAnchorPosition,
    .getAnchorIdList = getAnchorIdList,              // return the active id num: uint8_t
    .getActiveAnchorIdList = getActiveAnchorIdList,
};

// Add inter-drone range logging
LOG_GROUP_START(tdoa4)
LOG_ADD(LOG_FLOAT, inter_range,  &ra_inputstate.inter_range)
LOG_ADD(LOG_INT16, rAgentID,     &log_rAgentID)
// LOG_ADD(LOG_FLOAT, delay, &sam_temp)
// // Sam
// LOG_ADD(LOG_FLOAT, v_xj, &ra_inputstate.remote_vx)
// LOG_ADD(LOG_FLOAT, v_yj, &ra_inputstate.remote_vy)
// LOG_ADD(LOG_FLOAT, rj, &ra_inputstate.remote_gz)
// LOG_ADD(LOG_FLOAT, h_j, &ra_inputstate.remote_h)

LOG_GROUP_STOP(tdoa4)

// Logged communication





