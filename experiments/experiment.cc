/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "experiment.h"

#include "ns3/core-module.h"
#include "ns3/helper-module.h"
#include "../context-map.h"
#include "cws-measurement.h"

// static Time GetGlobalTime(std::string name) {
//   TimeValue tv;
//   GlobalValue::GetValueByName(name, tv);
//   return tv.Get();
// }

static ns3::GlobalValue g_phyDataRate ("PhyDataRate", "The global data rate in Mbps.",
                                       UintegerValue (6), MakeUintegerChecker<uint32_t> ());

NS_OBJECT_ENSURE_REGISTERED (Experiment);
NS_LOG_COMPONENT_DEFINE ("Experiment");

TypeId 
Experiment::GetTypeId(void )
{
  static TypeId tid = TypeId("Experiment")
    .SetParent<Object> ()
    .AddConstructor<Experiment> ()
    .AddAttribute ("nc", "Number of nodes", 
                   UintegerValue (10),
                   MakeUintegerAccessor(&Experiment::m_nodesCount),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("duration", "Simulation duration (eg: '10s'). Set to '0s' for no limit", 
                   TimeValue (MilliSeconds(0)),
                   MakeTimeAccessor(&Experiment::m_simDuration),
                   MakeTimeChecker())
    .AddAttribute ("mstart", "Measurement start time.", 
                   TimeValue (Seconds(1.0f)),
                   MakeTimeAccessor(&Experiment::measurementStart),
                   MakeTimeChecker())
    .AddAttribute ("mend", "Measurement end time.", 
                   TimeValue (MilliSeconds(0)),
                   MakeTimeAccessor(&Experiment::measurementEnd),
                   MakeTimeChecker())
    ;
  return tid;
}

Experiment::Experiment() : 
    mdata(0), measurementStart(), measurementEnd(), 
    m_nodesCount(), m_esmProtocol(), m_rsmProtocol(), m_protocols(0), m_simDuration()
{ }

Experiment::~Experiment() { }
//     viz(false), simDuration(NanoSeconds(0)), wpm(wpm_), rts(rts_), mdata(0),
//     measurementStart(Seconds(1)), measurementEnd(Seconds(2))

void Experiment::AddWsmProtocol(std::string otherWsmp)
{
  m_protocols.push_back(otherWsmp);
}

void Experiment::SetEsmProtocol(std::string esmp)
{
  m_esmProtocol = esmp;
}

void Experiment::SetRsmProtocol(std::string rsmp)
{
  m_rsmProtocol = rsmp;
}

void Experiment::Run()
{
  NS_LOG_DEBUG("[Experiment::Run] Start");
  NodeContainer nodes = NodeContainer();
  //setup wifi
  //setup rts
  m_rts = CreateRTS();
  if (m_rts == 0) {
    NS_LOG_WARN ("[Experiment::Run] Terminated. Override Experiment::CreateRTS() to not return 0.");
    return;
  }
  if (m_rts->ExpectedVehiclesCount() == 0) {
    NS_LOG_WARN ("[Experiment::Run] Terminated. No node created.");
    return;
  }
  //NS_ASSERT_MSG( m_rts != 0, "Override Experiment::CreateRTS() to not return 0." );
  //NS_LOG_UNCOND(m_rts->ExpectedVehiclesCount() << " " << m_nodesCount);
  //NS_ASSERT( m_rts->ExpectedVehiclesCount() == m_nodesCount );
  m_rts->Install(nodes);
  //NS_ASSERT( nodes.GetN() == m_nodesCount );
  NS_ASSERT( nodes.GetN() == m_rts->VehiclesCount() );
  m_rts->SetNotifyCollision(MakeCallback(&Experiment::CollisionTrace, this));
  m_rts->SetNotifyAccelerationChange(MakeCallback(&Experiment::SendEsm, this));
  m_rts->SetNotifyActiveStatusChange(MakeCallback(&Experiment::DeactivateNode, this));
  
  InstallWaveDevice(nodes);
  SetupMeasurement(nodes);
    
  //install context map
  //ContextMap::Install(nodes);

  //install wsm protocols
  //InstallWsmProtocols(nodes);
  
  m_rts->AllowsStopSimulation(true); //allows simulation to stop if there are no movement
  if (m_simDuration.IsStrictlyPositive()) {
    Simulator::Stop(m_simDuration);
    m_rts->AllowsStopSimulation(false);
  }
  
  Simulator::Run ();
  NS_LOG_INFO("[Experiment::Run] Finish @" << Simulator::Now());
  DumpCurrentVehicleState(nodes);
  PrintOutput();
  m_rts = 0;
  Simulator::Destroy ();

}

void Experiment::InstallWsmProtocols(NodeContainer& nodes)
{
  EsmProtocol::SetRecvCallback(MakeCallback(&Experiment::EsmReceived, this));
  EsmProtocol::SetSendCallback(MakeCallback(&CwsMeasurement::UpdateWarningSent, &mdata));
  RsmProtocol::SetRecvCallback( MakeCallback(&CwsMeasurement::UpdateBeaconReceived, &mdata) );
  RsmProtocol::SetSendCallback( MakeCallback(&CwsMeasurement::UpdateBeaconSent, &mdata) );
  ObjectFactory esmpFactory;
  esmpFactory.SetTypeId (m_esmProtocol);
  ObjectFactory rsmpFactory;
  rsmpFactory.SetTypeId (m_rsmProtocol);
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i) {
    Ptr<Node> node = *i;
    Ptr<EsmProtocol> esmp = esmpFactory.Create <EsmProtocol> ();
    esmp->Install(node);
    Ptr<RsmProtocol> rsmp = rsmpFactory.Create <RsmProtocol> ();
    rsmp->Install(node);
    //protocol->SetMaxTxPowerLevel(19);
    //protocol->SetNotifyFirstEsmReceived(MakeCallback(&Experiment::EsmReceived, this)); //first msg received
  }
  
  for (uint32_t i=0; i<m_protocols.size(); ++i) {
    ObjectFactory factory;
    factory.SetTypeId (m_protocols[i]);
    for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i) {
      Ptr<Node> node = *i;
      Ptr<WsmProtocol> protocol = factory.Create <WsmProtocol> ();
      protocol->Install(node);
    }    
  }
}

void Experiment::SetSimulationDuration(Time dur) {
  m_simDuration = dur;
}

void Experiment::SetMeasurementDuration(Time start, Time end) {
  NS_ASSERT(!start.IsNegative());
  NS_ASSERT(!end.IsNegative());
  if (end.IsZero()) end = m_simDuration;
  NS_ASSERT(start < end);
  measurementStart = start;
  measurementEnd = end;
}

void Experiment::DumpCurrentVehicleState(const NodeContainer& nodes) const {
  NS_LOG_INFO("[Experiment::DumpCurrentVehicleState]");
  for (uint32_t i=0; i<nodes.GetN(); ++i) {
    Ptr<VehicleMobilityModel> m = nodes.Get(i)->GetObject<VehicleMobilityModel>();
    //" indexAtLane=" << m->GetIndexAtLane() << 
    Ptr<VehicleMobilityModel> ml = m->GetLeader();
    int64_t leaderId = -1;
    if (ml) {
      Ptr<Node> n = ml->GetObject<Node>();
      leaderId = n->GetId();
    }
    NS_LOG_INFO("nodeId="<< i << " laneId=" << m->GetLaneId() << " leader=" << leaderId << 
                  " a=" << m->GetAcceleration() << " cv=" << mdata.GetCollisionSpeed(i) << " v=" << m->GetSpeed() <<
                  " p=" << m->GetPosition() << " o=" << m->GetOffsetAlongPath() << 
                  " delay=" << mdata.GetWarningDelay(i) << 
                  " W-sent=" << mdata.GetWarningSentCount(i) <<
                  " W-recv=" << mdata.GetWarningReceivedCount(i) 
               );
    //Ptr<ContextMap> cm = nodes.Get(i)->GetObject<ContextMap>();
    //for (ContextMap::TStateMap::const_iterator it=cm->stateMap.begin(); it!=cm->stateMap.end(); it++) {
    //  NS_LOG_INFO("  nodeId="<< it->first << " t=" << it->second.timestamp.GetSeconds() << " " << it->second.state);
    //}
  }
}

void Experiment::PhyStateTrace(Time start, Time duration, WifiPhy::State state) {
  if (start >= measurementStart && start < measurementEnd) {
    //NS_LOG_DEBUG ("start=" << start << ", dur=" << duration << ", state=" << state);
    if (state != WifiPhy::IDLE) {
      mdata.AccumulateChannelBusyTime(duration);
    }
  }
}

void Experiment::CollisionTrace(Ptr< Node > node, double speedAtCollision, double speedDiff) {
  mdata.UpdateCollision(node->GetId(), speedDiff);
}


void Experiment::MeasureDistanceError(const ns3::NodeContainer& nodes) {
  if (Simulator::Now() < measurementEnd) {
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
      Ptr<Node> n = nodes.Get(i);
      Ptr<ContextMap> cm = n->GetObject<ContextMap>();
      cm->CalculateDistanceError(MakeCallback(&CwsMeasurement::AccumulateDistanceError, &mdata));
    }
    Simulator::Schedule( MilliSeconds(50), &Experiment::MeasureDistanceError, this, nodes );
  }
}

void Experiment::PrintOutput() const {
/*  double minLatency = 99999999.0f;
  double maxLatency = 0.0f;
  double avgLatency = 0.0f;
  uint32_t totalNotReceivingNodes = 0;
  mdata.GetWarningDelayStat(avgLatency, minLatency, maxLatency, totalNotReceivingNodes);
  
  // Output result
  if (outputFlags.totalCollisions)
    std::cout << mdata.GetTotalCollisions() << " ";
  if (outputFlags.totalWarningSent)
    std::cout << mdata.GetTotalWarningSent() << " ";
  if (outputFlags.totalNotReceivingNodes)
    std::cout << totalNotReceivingNodes << " "; //nodes not receive
  if (outputFlags.minLatency)
    std::cout << minLatency << " ";
  if (outputFlags.maxLatency)
    std::cout << maxLatency << " ";
  if (outputFlags.avgLatency)
    std::cout << avgLatency << " ";
  if (outputFlags.channelBusyTime)
    std::cout << mdata.GetChannelUsageLevel() << " ";
  std::cout << std::endl;
  if (outputFlags.nodesLatency) {
    std::cout << "latency: ";
    for (uint32_t i=0; i<mdata.NodesCount(); ++i) {
      std::cout << mdata.GetWarningDelay(i) << " ";
    }
    std::cout << std::endl;
  }    
  if (outputFlags.nodesSpeedDiff) {
    std::cout << "speed: ";
    for (uint32_t i=0; i<mdata.NodesCount(); ++i) {
      std::cout << mdata.GetCollisionSpeed(i) << " ";
    }
    std::cout << std::endl;
  }    */
}

void Experiment::EsmReceived(uint32_t receiverId, uint32_t messageId, Time delay)
{
  mdata.UpdateWarningReceived(receiverId, messageId, delay.GetSeconds());
  // schedule brake
  m_rts->InitiateBraking(NodeList::GetNode(receiverId));
}

void Experiment::SendEsm(Ptr< Node > sender, double oldAccel, double newAccel)
{
  NS_LOG_DEBUG("[Experiment::SendEsm]");
  if (newAccel <= (-m_rts->pVehicleAbnormalDecel) ) {
    Ptr<EsmProtocol> esmp = sender->GetObject<EsmProtocol>();
    esmp->SendInitialWarning();
    //SendInitialWarning(node);
  }
}

void Experiment::DeactivateNode(Ptr< VehicleMobilityModel > vmm, bool isActive)
{
  if (!isActive) {
    Ptr<Node> n = vmm->GetObject<Node>();
    Object::AggregateIterator ai = n->GetAggregateIterator();
    while (ai.HasNext()) {
      Ptr<Object> o = ConstCast<Object>( ai.Next() );
      Ptr< WsmProtocol > wsmp =  DynamicCast< WsmProtocol > ( o );
      //Ptr< WsmProtocol > wsmp =  o->GetObject< WsmProtocol >();
      if (wsmp != 0) {
        wsmp->Deactivate();
      }
    }
  }
}

void Experiment::InstallWaveDevice(NodeContainer& nodes) const
{
  NS_ASSERT ( nodes.GetN() != 0 );
  // disable fragmentation for frames below 22000 bytes
  //Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("22000"));
  // turn off RTS/CTS for frames below 22000 bytes
  //Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("22000"));
  Config::SetDefault("ns3::YansWifiPhy::TxPowerStart", DoubleValue(0));
  Config::SetDefault("ns3::YansWifiPhy::TxPowerEnd", DoubleValue(19));
  Config::SetDefault("ns3::YansWifiPhy::TxPowerLevels", UintegerValue(20));
  
  // Fix non-unicast data rate to be the same as that of unicast
  uint32_t datarate;
  UintegerValue value;
  g_phyDataRate.GetValue(value);
  datarate = value.Get();
  std::stringstream sstemp;
  sstemp << "OfdmRate" << datarate << "MbpsBW10MHz";
  std::string phyMode = sstemp.str();
  
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
  WifiHelper wifiHelper = WifiHelper::Default();
  wifiHelper.SetStandard( WIFI_PHY_STANDARD_80211p_CCH );
  //disable rate control
  wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue(phyMode),
                                      "ControlMode", StringValue(phyMode),
                                      "NonUnicastMode", StringValue (phyMode)
                                    );
                                       
  YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default();
    
  YansWifiChannelHelper wifiChannelHelper;
  wifiChannelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannelHelper.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel");
  wifiChannelHelper.AddPropagationLoss("ns3::NakagamiPropagationLossModel", 
                                         "m0", DoubleValue(1.0), "m1", DoubleValue(1.0), "m2", DoubleValue(1.0));
    
  // Add a upper mac
  QosWifiMacHelper wifiMacHelper = QosWifiMacHelper::Default();
  // Set it to adhoc mode
  //wifiMacHelper.SetType("ns3::QadhocWifiMac");
  wifiMacHelper.SetType("ns3::AdhocWifiMac");
  /// Install wifi L1 and L2
  wifiPhyHelper.SetChannel( wifiChannelHelper.Create() );
  NetDeviceContainer devices = wifiHelper.Install (wifiPhyHelper, wifiMacHelper, nodes);

}

void Experiment::SetupMeasurement(NodeContainer& nodes)
{
  mdata = CwsMeasurement(nodes.GetN());
  mdata.SetDuration(measurementStart, measurementEnd);
  //Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChangeCallback));
  //Measure channel busy time
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/0/Phy/State/State", MakeCallback (&Experiment::PhyStateTrace, this));
  //Measure distance error
  Simulator::Schedule( measurementStart, &Experiment::MeasureDistanceError, this, nodes );
}

Ptr< RoadTrafficScenario > Experiment::CreateRTS() const
{
  return 0;
}

/*
void Experiment::SetOutput(OutputFlags flags) {
  m_outputFlags = flags;
}
*/

