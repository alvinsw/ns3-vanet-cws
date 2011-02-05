#ifndef DEF_PARAM_H
#define DEF_PARAM_H

#include "ns3/nstime.h"
#include "road-traffic-scenario.h"
#include "context-map.h"

using namespace ns3;

const double VEHICLE_MIN_GAP = 2.0f;
const double DISTANCE_EPSILON = 0.001f; //the epsilon is set to 1 millimeter
//default values
const float VEHICLE_W = 2.0f;
const float VEHICLE_L = 4.0f;
/** vehicle speed in m/s */
const float VEHICLE_SPEED = 32.0f;
/** vehicle emergency deceleration in m/s^2, used by the first node */
const float VEHICLE_MAX_DECEL = 8.0f;
/** vehicle regular deceleration in m/s^2, used by the nodes except first node */
const float VEHICLE_DECEL = 4.9f;
/** Highway length in meters */
const float HW_LENGTH = 500;
/** Highway width in meters */
const float HW_LANE_WIDTH = 4;
const int HW_LANES_NUM = 10;
const uint32_t NODES_NUM = 50;
/** inter-vehicle spacing in meter */
const float IV_SPACING = 16.0f;
const Time REACTION_TIME_MIN = Seconds(1.5);
const Time REACTION_TIME_MAX = Seconds(1.5);

const Time MAX_SIM_DURATION = Seconds(60);
const Time TIME_OF_ACCIDENT = Seconds(5);
const Time MOVEMENT_UPDATE_PERIOD = MilliSeconds(50);

/** Transmission power used to broadcast the warning message, only used in a fixed-tx-range protocols */
const double DP_TX_POWER = 19.0f;  // power 15 give 600 m range, 27 give 300 m
const int DP_PHY_DATA_RATE = 6;

/** Warning message packet size in bytes */
const unsigned int DP_ESM_SIZE = 500;
const Time DP_ESM_INTERVAL = MilliSeconds(100);
const uint32_t DUMMY_TRAFFIC_SIZE = 0;
const uint32_t DUMMY_TRAFFIC_RATE = 0;
const uint32_t DP_RSM_SIZE = 500;
const Time DP_RSM_INTERVAL = MilliSeconds(100);

////IBIA default parameters 
/** IBIA maximum random wait time [0 to IBIA_MAX_WAIT] inclusive */
const Time IBIA_MAX_WAIT = MilliSeconds(10);
const uint32_t IBIA_MAX_REPETITION = 10;

////EMDV default parameters 
const double EMDV_DISS_AREA_LENGTH = 4000; //in meters
const double EMDV_FW_RANGE = 500;
const int32_t EMDV_MAX_MESSAGES = 3;
const Time EMDV_MAX_CONTENTION_TIME = MilliSeconds(100);
const Time EMDV_MAX_CHANNEL_ACCESS_TIME = MilliSeconds(10);

////M0 default parameters 
const uint32_t M0_MAX_REPETITION = 5;
const uint32_t M0_RELAY_WAIT_INC = 1; //relay wait time increament step in millisecond
const Time M0_REPEAT_INTERVAL = MilliSeconds(10);

struct Parameters {
  Parameters() {
    RoadTrafficScenario::defaultVehicleAbnormalDecel = VEHICLE_MAX_DECEL;
    RoadTrafficScenario::defaultVehicleNormalDecel = VEHICLE_DECEL;
    RoadTrafficScenario::defaultReactionTimeMin = REACTION_TIME_MIN;
    RoadTrafficScenario::defaultReactionTimeMax = REACTION_TIME_MAX;
    RoadTrafficScenario::defaultMinGap = 1.0f;
    RoadTrafficScenario::defaultUpdatePeriod = MilliSeconds(20);
    RoadTrafficScenario::defaultDistanceEpsilon = DISTANCE_EPSILON;
    
    ContextMap::defaultMinGap = 2.0f;
    ContextMap::defaultVehicleDecelMax = VEHICLE_MAX_DECEL;
    ContextMap::defaultVehicleDecelNormal = VEHICLE_DECEL;
    ContextMap::defaultReactionTimeMax = REACTION_TIME_MAX;
    //BeaconProtocol::defaultPriority = PRIORITY_BEACON;
  }
};
#endif /* DEF_PARAM_H */
