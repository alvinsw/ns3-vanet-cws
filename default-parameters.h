#ifndef DEF_PARAM_H
#define DEF_PARAM_H

#include "ns3/nstime.h"

using namespace ns3;

const double DP_VEH_MIN_GAP = 2.0f;
const double DP_DISTANCE_EPSILON = 0.001f; //the epsilon is set to 1 millimeter
//default values
const float DP_VEH_W = 2.0f;
const float DP_VEH_L = 4.0f;
/** vehicle speed in m/s */
const float DP_VEH_SPEED = 32.0f;
/** vehicle emergency deceleration in m/s^2, used by the first node */
const float DP_VEH_MAX_DECEL = 8.0f;
/** vehicle regular deceleration in m/s^2, used by the nodes except first node */
const float DP_VEH_DECEL = 4.9f;
/** Highway length in meters */
const float DP_HW_LENGTH = 500;
/** Highway width in meters */
const float DP_HW_LANE_WIDTH = 4;
const int DP_HW_LANES_NUM = 10;
const uint32_t DP_NODES_NUM = 50;
/** inter-vehicle spacing in meter */
const float DP_IV_SPACING = 16.0f;
const Time DP_REACTION_TIME_MIN = Seconds(1.5);
const Time DP_REACTION_TIME_MAX = Seconds(1.5);

const Time DP_MAX_SIM_DURATION = Seconds(60);
const Time DP_TIME_OF_ACCIDENT = Seconds(5);
const Time DP_MOVEMENT_UPDATE_PERIOD = MilliSeconds(50);

/** Transmission power used to broadcast the warning message, only used in a fixed-tx-range protocols */
const double DP_TX_POWER = 19.0f;  // power 15 give 600 m range, 27 give 300 m
const int DP_PHY_DATA_RATE = 6;

/** Warning message packet size in bytes */
const unsigned int DP_ESM_SIZE = 500;
const Time DP_ESM_INTERVAL = MilliSeconds(100);
const uint32_t DP_RSM_SIZE = 500;
const Time DP_RSM_INTERVAL = MilliSeconds(100);
const uint32_t DP_DUMMY_SIZE = 0;
const uint32_t DP_DUMMY_RATE = 0;

////IBIA default parameters 
/** IBIA maximum random wait time [0 to IBIA_MAX_WAIT] inclusive */
const Time DP_IBIA_MAX_WAIT = MilliSeconds(10);
const uint32_t DP_IBIA_REPEAT_COUNT = 10;

////EMDV default parameters 
const double DP_EMDV_DISS_AREA_LENGTH = 4000; //in meters
const double DP_EMDV_FW_RANGE = 500;
const int32_t DP_EMDV_MAX_MESSAGES = 3;
const Time DP_EMDV_MAX_CONTENTION_TIME = MilliSeconds(100);
const Time DP_EMDV_MAX_CHANNEL_ACCESS_TIME = MilliSeconds(10);

////M0 default parameters 
const uint32_t DP_M0_MAX_REPETITION = 5;
const uint32_t DP_M0_RELAY_WAIT_INC = 1; //relay wait time increament step in millisecond
const Time DP_M0_REPEAT_INTERVAL = MilliSeconds(10);

#endif /* DEF_PARAM_H */
