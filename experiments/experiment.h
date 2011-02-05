/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//TODO: fix: add callback in Run() rts.SetNotifyDecelerationChange(MakeCallback(&WarningProtocol::InitiateWarning, warningProtocol)

#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "ns3/command-line.h"
#include "ns3/wifi-module.h"
#include "../road-traffic-scenario.h"
#include "../protocols/esm-protocol.h"
#include "../protocols/rsm-protocol.h"
#include "cws-measurement.h"

using namespace ns3;

class Experiment : public Object {
  public:
    static TypeId GetTypeId(void);
    
    Experiment();
    virtual ~Experiment();
    //Experiment(RoadTrafficScenario& rts_, WifiPhyMacHelper& wpm_);

    void SetEsmProtocol(std::string esmp);
    void SetRsmProtocol(std::string rsmp);
    void AddWsmProtocol(std::string otherWsmp);
    //void SetOutput(OutputFlags flags);
    void SetSimulationDuration(Time dur);
    void SetMeasurementDuration(Time start, Time end);
   
    void Run();
    
    void MeasureDistanceError(const NodeContainer& nodes);
    void DumpCurrentVehicleState(const NodeContainer& nodes) const;
    virtual void PrintOutput() const;
    
    //Callbacks methods. Do not call these methods directly
    void SendEsm(Ptr<Node> sender, double oldAccel, double newAccel);
    void EsmReceived(uint32_t receiverId, uint32_t messageId, Time delay);
    void DeactivateNode(Ptr<VehicleMobilityModel> vmm, bool isActive);
    
  protected:
    virtual void InstallWaveDevice(NodeContainer& nodes) const;
    virtual void SetupMeasurement(NodeContainer& nodes);
    /** Setup RoadTrafficScenario that will be used in the experiment. */
    virtual Ptr<RoadTrafficScenario> CreateRTS() const;
    
    void PhyStateTrace(Time start, Time duration, enum WifiPhy::State state);
    void CollisionTrace(Ptr<Node> node, double speedAtCollision, double speedDiff);
    
    //RoadTrafficScenario& rts;
    CwsMeasurement mdata;
    Time measurementStart;
    Time measurementEnd;
private:
  void InstallWsmProtocols(NodeContainer& nodes);
  
  uint32_t m_nodesCount;
  std::string m_esmProtocol;
  std::string m_rsmProtocol;
  std::vector< std::string > m_protocols;
  Time m_simDuration;
  Ptr<RoadTrafficScenario> m_rts;
  //OutputFlags m_outputFlags;
};

/*
struct OutputFlags {
  OutputFlags() : 
      totalCollisions(true), totalWarningSent(true), totalNotReceivingNodes(true), 
      minLatency(false), maxLatency(false), avgLatency(true),
      channelBusyTime(false), nodesLatency(false), nodesSpeedDiff(false) {}
  bool totalCollisions;
  bool totalWarningSent;
  bool totalNotReceivingNodes;
  bool minLatency;
  bool maxLatency;
  bool avgLatency;
  bool channelBusyTime;
  bool nodesLatency;
  bool nodesSpeedDiff;
};
*/
#endif /* EXPERIMENT_H */
