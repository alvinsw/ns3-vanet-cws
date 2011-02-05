/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef CWS_MEASUREMENT_H
#define CWS_MEASUREMENT_H

#include <vector>
#include <map>
#include "ns3/nstime.h"
#include "limits"

using namespace ns3;

struct MDataWarning {
  MDataWarning(uint32_t sent = 0, uint32_t received = 0, double delay=std::numeric_limits<double>::infinity()) : 
      warningSent(sent), warningReceived(received), warningReceivedDelay(delay) {}
  uint32_t warningSent;
  uint32_t warningReceived;
  /** Warning reception delay calculated from the initial sender (when abnormal event was triggered), not from the immediate sender */
  double warningReceivedDelay;
};

/** Measurement data for each node */
struct MDataNode {
  MDataNode(uint32_t distanceSteps);
  uint32_t warningSentCount;
  uint32_t warningReceivedCount;
  std::map< uint32_t, MDataWarning > warnings;
  uint32_t beaconSentCount;
  //std::vector< uint32_t > beaconReceivedCount; //index is the sender
  std::vector< double > beaconReceivedProb;
  std::vector< uint32_t > vehiclesCountInArea;
  //uint64_t receivedPacketSize;
  //Time channelBusyTime;
  double collisionSpeed;
};

class CwsMeasurement {
  public:
    CwsMeasurement(uint32_t nodesCount);
      
    void InitWarning(uint32_t nodeId, uint32_t warningId);
    void SetDuration(Time start, Time end);
    
    void UpdateWarningSent(uint32_t nodeId, uint32_t warningId);
    void UpdateWarningReceived(uint32_t nodeId, uint32_t warningId, double delayInSeconds);
    void UpdateBeaconSent(uint32_t senderId, Time interval, uint32_t msgId);
    void UpdateBeaconReceived(uint32_t senderId, uint32_t receiverId, Time delay, uint32_t msgId);
    void UpdateCollision(uint32_t nodeId, double collisionSpeed);
    
    void AccumulateDistanceError(double distanceError);
    void AccumulateChannelBusyTime(Time t);
    
    uint32_t DistanceStepsCount() const;
    uint32_t NodesCount() const;
    /** Average beacon reception probability*/
    double GetAvgBeaconRP(uint32_t distanceIndex) const;
    double GetAvgDistanceError() const;
    double GetMinDistanceError() const;
    double GetMaxDistanceError() const;
    double GetAvgBeaconInterval() const;
    double GetAvgBeaconDelay() const;
    double GetWarningDelay(uint32_t nodeId) const;
    void GetWarningDelayStat(double& avg, double& min, double& max, uint32_t nonReceivingNodes) const;
    const MDataWarning& GetWarningStat(uint32_t nodeId, uint32_t mid) const;
    double GetChannelUsageLevel() const;
    uint32_t GetWarningSentCount(uint32_t nodeId) const;
    uint32_t GetWarningReceivedCount(uint32_t nodeId) const;
    uint32_t GetTotalWarningSent() const;
    uint32_t GetTotalBeaconSent() const;
    uint32_t GetTotalCollisions() const;
    double GetCollisionSpeed(uint32_t nodeId) const;
  
  private:
    static const uint32_t DIST_STEPS_COUNT = 20;
    static const uint32_t MAX_DIST = 1000;
    static const uint32_t DIST_STEP = MAX_DIST/DIST_STEPS_COUNT;
    
    Time measurementStart;
    Time measurementEnd;
    std::vector< MDataNode > nodes;
    //uint32_t totalWarningSent;
    //uint32_t totalAckSent;
    //Time simDuration;
    uint32_t totalCollisions;
    Time channelBusyTime;
    
    double distanceErrorSum;
    double distanceErrorMin;
    double distanceErrorMax;
    double beaconIntervalSum;
    double beaconDelaySum;
    uint32_t distanceErrorSumCount;
    uint32_t beaconIntervalSumCount;
    uint32_t beaconDelaySumCount;
  
};

#endif // CWS_MEASUREMENT_H
