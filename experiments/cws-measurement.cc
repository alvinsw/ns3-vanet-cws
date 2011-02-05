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

#include "cws-measurement.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"


MDataNode::MDataNode(uint32_t distanceSteps)  : 
      warningSentCount(0), warningReceivedCount(0),
      //latency(), 
      warnings(), 
      beaconSentCount(0), //beaconReceivedCount(count), 
      beaconReceivedProb(distanceSteps, -1.0f), vehiclesCountInArea(distanceSteps, 0),
      //receivedPacketSize(0), channelBusyTime(), 
      collisionSpeed(0.0f)
{}

CwsMeasurement::CwsMeasurement(uint32_t nodesCount) : 
      measurementStart(), measurementEnd(), 
      nodes(nodesCount, MDataNode(DIST_STEPS_COUNT)), 
      //totalWarningSent(0), 
      //totalAckSent(0), 
      //simDuration( NanoSeconds(0) ),
      totalCollisions(0) ,
      channelBusyTime( NanoSeconds(0) ),
      distanceErrorSum(0.0f), distanceErrorMin(1000.0f), distanceErrorMax(0.0f), 
	  beaconIntervalSum(0), beaconDelaySum(0), 
      distanceErrorSumCount(0), beaconIntervalSumCount(0), beaconDelaySumCount(0)
{}


void CwsMeasurement::InitWarning(uint32_t nodeId, uint32_t warningId) {
  nodes[nodeId].warnings[warningId] = MDataWarning(0,0);
}

void CwsMeasurement::SetDuration(Time start, Time end) {
  measurementStart = start;
  measurementEnd = end;
}


void CwsMeasurement::UpdateBeaconReceived(uint32_t senderId, uint32_t receiverId, Time delay, uint32_t msgId) {
  Time now = Simulator::Now();
  if (now >= measurementStart && now <= measurementEnd) {
    //nodes[receiverId].beaconReceived[senderId]++;
    beaconDelaySum += delay.GetSeconds();
    beaconDelaySumCount++;
    
    Ptr<MobilityModel> m0 = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    Ptr<MobilityModel> m = NodeList::GetNode(receiverId)->GetObject<MobilityModel>();
    double dist = m->GetDistanceFrom(m0);
    uint32_t index = (uint32_t) (dist / DIST_STEP);
    if (index < DIST_STEPS_COUNT && nodes[senderId].vehiclesCountInArea[index] > 0) {
      //std::cout<< "nodes["<<senderId<<"].vehiclesCountInArea["<<index<<"]=" <<nodes[senderId].vehiclesCountInArea[index]<<std::endl;
      nodes[senderId].beaconReceivedProb[index] += (1.0f/(double)nodes[senderId].vehiclesCountInArea[index]);
      //std::cout<< "nodes["<<senderId<<"].beaconReceivedProb["<<index<<"]=" <<nodes[senderId].beaconReceivedProb[index]<<std::endl;
    }
  }
}

void CwsMeasurement::UpdateBeaconSent(uint32_t senderId, Time interval, uint32_t msgId) {
  Time now = Simulator::Now();
  if (now >= measurementStart && now <= measurementEnd) {
    nodes[senderId].beaconSentCount++;
    beaconIntervalSum += interval.GetSeconds();
    beaconIntervalSumCount++;
    
    nodes[senderId].vehiclesCountInArea.assign(DIST_STEPS_COUNT, 0);
    Ptr<MobilityModel> m0 = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    for (uint32_t i=0; i<senderId; ++i) {
      Ptr<MobilityModel> m = NodeList::GetNode(i)->GetObject<MobilityModel>();
      double dist = m->GetDistanceFrom(m0);
      uint32_t index = (uint32_t) (dist / DIST_STEP);
      if (index < DIST_STEPS_COUNT ) {
        nodes[senderId].vehiclesCountInArea[index]++;
        if (nodes[senderId].beaconReceivedProb[index] < 0.0f) nodes[senderId].beaconReceivedProb[index] = 0.0f;
      }
    }
    for (uint32_t i=senderId+1; i<NodeList::GetNNodes(); ++i) {
      Ptr<MobilityModel> m = NodeList::GetNode(i)->GetObject<MobilityModel>();
      double dist = m->GetDistanceFrom(m0);
      uint32_t index = (uint32_t) (dist / DIST_STEP);
      if (index < DIST_STEPS_COUNT) {
        nodes[senderId].vehiclesCountInArea[index]++;
        if (nodes[senderId].beaconReceivedProb[index] < 0.0f) nodes[senderId].beaconReceivedProb[index] = 0.0f;
      }
    }
  }
}
void CwsMeasurement::UpdateWarningReceived(uint32_t nodeId, uint32_t warningId, double delayInSeconds)
{
  MDataWarning& esm = nodes[nodeId].warnings[warningId];
  esm.warningReceived++;
  if (esm.warningReceivedDelay > delayInSeconds) {
    esm.warningReceivedDelay = delayInSeconds;
  }
  nodes[nodeId].warningReceivedCount++;
}

void CwsMeasurement::UpdateWarningSent(uint32_t nodeId, uint32_t warningId) {
  nodes[nodeId].warnings[warningId].warningSent++;
  nodes[nodeId].warningSentCount++;
  //totalWarningSent++;
}

void CwsMeasurement::UpdateCollision(uint32_t nodeId, double collisionSpeed) {
  nodes[nodeId].collisionSpeed = collisionSpeed;
  totalCollisions++;
}

void CwsMeasurement::AccumulateDistanceError(double distanceError) {
  if (distanceErrorMin > distanceError) distanceErrorMin = distanceError;
  if (distanceErrorMax < distanceError) distanceErrorMax = distanceError;
  distanceErrorSum += distanceError;
  distanceErrorSumCount++;
}

void CwsMeasurement::AccumulateChannelBusyTime(Time t) {
  channelBusyTime += t;
}

uint32_t CwsMeasurement::DistanceStepsCount() const {
  return DIST_STEPS_COUNT;
}

uint32_t CwsMeasurement::NodesCount() const {
  return nodes.size();
}

double CwsMeasurement::GetAvgBeaconRP(uint32_t distanceIndex) const {
  double rp = 0;
  uint32_t count = 0;
  for (uint32_t i=0; i<nodes.size(); ++i) {
    //std::cout << "nodes["<<i<<"].beaconReceivedProb["<<distanceIndex<<"]=" << nodes[i].beaconReceivedProb[distanceIndex] << std::endl;
    if (nodes[i].beaconReceivedProb[distanceIndex] >= 0.0f) {
      rp += (nodes[i].beaconReceivedProb[distanceIndex]/nodes[i].beaconSentCount);
      count++;
    }
  }
  if (count > 0) {
    return rp/(double)count;
  }
  return -1.0f;
  //std::cout << rp << " ";
}

double CwsMeasurement::GetAvgDistanceError() const {
  return distanceErrorSum/distanceErrorSumCount;
}
double CwsMeasurement::GetMinDistanceError() const {
  return distanceErrorMin;
}
double CwsMeasurement::GetMaxDistanceError() const {
  return distanceErrorMax;
}

double CwsMeasurement::GetAvgBeaconInterval() const {
  return beaconIntervalSum/beaconIntervalSumCount;
}

double CwsMeasurement::GetAvgBeaconDelay() const {
  return beaconDelaySum/beaconDelaySumCount;
}

double CwsMeasurement::GetWarningDelay(uint32_t nodeId) const {
  double total = 0.0f;
  int count = 0;
  std::map< uint32_t, MDataWarning >::const_iterator it;
  for (it = nodes[nodeId].warnings.begin(); it != nodes[nodeId].warnings.end(); ++it) {
    if (it->second.warningReceivedDelay < std::numeric_limits< double >::infinity()) {
      total += it->second.warningReceivedDelay;
      count++;
    }
  }
  if (count > 0) return total/(double)count;
  else return 0.0f;
}

void CwsMeasurement::GetWarningDelayStat(double& avg, double& min, double& max, uint32_t nonReceivingNodes) const {
  //Calculate avg latency
  uint32_t rcount = 0;
  double totalLatency = 0.0f;
  double minLatency = 99999999.0f;
  double maxLatency = 0.0f;
  for (uint32_t i=0; i<nodes.size(); ++i) {
    //if (m.n[i].warningReceivedTime.IsZero()) {    }
    double l = GetWarningDelay(i);
    if (l > 0.0f) { //node receive warning
      rcount++;
      totalLatency += l;
      if (minLatency > l) minLatency = l;
      if (maxLatency < l) maxLatency = l;
    }
  }
  avg = totalLatency / (double)rcount;
  min = minLatency;
  max = maxLatency;
  nonReceivingNodes = nodes.size() - rcount;
}

const MDataWarning& CwsMeasurement::GetWarningStat(uint32_t nodeId, uint32_t mid) const {
  return nodes[nodeId].warnings.at(mid);
}

double CwsMeasurement::GetChannelUsageLevel() const {
  return (channelBusyTime.GetSeconds() / (double)nodes.size()) / (measurementEnd.GetSeconds() - measurementStart.GetSeconds());
}

uint32_t CwsMeasurement::GetWarningSentCount(uint32_t nodeId) const {
  return nodes[nodeId].warningSentCount;
}

uint32_t CwsMeasurement::GetWarningReceivedCount(uint32_t nodeId) const {
  return nodes[nodeId].warningReceivedCount;
}

uint32_t CwsMeasurement::GetTotalBeaconSent() const {
  uint32_t total=0;
  for (uint32_t i=0; i<nodes.size(); ++i) {
    total += nodes[i].beaconSentCount;
  }
  return total;
}

uint32_t CwsMeasurement::GetTotalWarningSent() const {
  uint32_t total = 0;
  for (uint32_t i=0; i<nodes.size(); ++i) {
    total += nodes[i].warningSentCount;
  }
  return total;
}

uint32_t CwsMeasurement::GetTotalCollisions() const {
  return totalCollisions;
}

double CwsMeasurement::GetCollisionSpeed(uint32_t nodeId) const {
  return nodes[nodeId].collisionSpeed;
}



