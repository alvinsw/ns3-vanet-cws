/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/global-value.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"

//#include "ns3/packet.h"
#include "dsrc-bsm-header.h"
#include "rsm-protocol.h"
#include "../default-parameters.h"

NS_OBJECT_ENSURE_REGISTERED (RsmProtocol);
NS_LOG_COMPONENT_DEFINE ("RsmProtocol");

static ns3::GlobalValue g_rsmIntervalMin ("RsmIntervalMin", "Minimum rsm interval.",
                                          TimeValue ( MilliSeconds(50) ), MakeTimeChecker ());
static ns3::GlobalValue g_rsmIntervalMax ("RsmIntervalMax", "Maximum rsm interval.",
                                          TimeValue ( MilliSeconds(1000) ), MakeTimeChecker ());
static ns3::GlobalValue g_rsmInterval ("RsmInterval", "Constant rsm interval.",
                                          TimeValue ( MilliSeconds(100) ), MakeTimeChecker ());
static ns3::GlobalValue g_rsmStartTime ("RsmStart", "Rsm start time.",
                                          TimeValue ( MilliSeconds(0) ), MakeTimeChecker ());
static ns3::GlobalValue g_rsmStopTime ("RsmStop", "Rsm stop time.",
                                          TimeValue ( MilliSeconds(0) ), MakeTimeChecker ());
static ns3::GlobalValue g_rsmJitterInterval ("RsmJitterInterval", "Maximum jitter interval.",
                                          TimeValue ( MilliSeconds(0) ), MakeTimeChecker ());
static ns3::GlobalValue g_rsmRandomStart ("RsmRandomStart", "Use random start time.",
                                          BooleanValue (true), MakeBooleanChecker ());
static ns3::GlobalValue g_rsmSize ("RsmSize", "Constant rsm packet size.",
                                          UintegerValue (DP_RSM_SIZE), MakeUintegerChecker<uint32_t> ());
//uint8_t RsmProtocol::pPriority = 5;
Time RsmProtocol::s_startTime = Seconds(0);
Time RsmProtocol::s_stopTime = Seconds(0);
RsmProtocol::RecvCallback RsmProtocol::s_recvCallback;
RsmProtocol::SendCallback RsmProtocol::s_sendCallback;
uint32_t RsmProtocol::s_constantSize;

TypeId 
RsmProtocol::GetTypeId(void) {
  static TypeId tid = TypeId("RsmProtocol")
    .SetParent<WsmProtocol> ()
    .AddConstructor<RsmProtocol> () 
    ;
  return tid;
}

void RsmProtocol::SetRecvCallback(RsmProtocol::RecvCallback cb)
{
  s_recvCallback = cb;
}
void RsmProtocol::SetSendCallback(RsmProtocol::SendCallback cb)
{
  s_sendCallback = cb;
}
void RsmProtocol::Start(Time time) {
  s_startTime = Simulator::Now() + time;
}

void RsmProtocol::Stop(Time time)
{
  s_stopTime = Simulator::Now() + time;
}

RsmProtocol::RsmProtocol() : WsmProtocol(RSMP_NUMBER, RSMP_PRIORITY), m_lastTransmission(Seconds(0)), m_nextTransmission()
{ 
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    TimeValue tv;
    g_rsmStartTime.GetValue (tv);
    s_startTime = tv.Get ();
    g_rsmStopTime.GetValue (tv);
    s_stopTime = tv.Get ();
    UintegerValue uv;
    g_rsmSize.GetValue (uv);
    s_constantSize = uv.Get ();
  }
}

RsmProtocol::~RsmProtocol() { }


// void RsmProtocol::SetNotifyPacketReceived(RsmProtocol::PacketReceivedCallback callback)
// {
//   notifyBeaconReceived = callback;
// }

// void RsmProtocol::SetInterval(Time interval) {
//   SetInterval(interval, interval);
// }
// void RsmProtocol::SetSize(uint32_t size) {
//   SetSize(size, size);
// }
// void RsmProtocol::SetInterval(Time min, Time max) { }
// void RsmProtocol::SetSize(uint32_t min, uint32_t max) { }
// uint32_t RsmProtocol::GetMaxSize() const {
//   return 0;
// }
// Time RsmProtocol::GetMinInterval() const {
//   return Seconds(0.0f);
// }

void RsmProtocol::InitiateSending()
{
  Time t = CalculateStartTime();
  if (t.IsPositive()) {
    t += s_startTime;
    Simulator::Schedule (t, &RsmProtocol::Send, this);
  }
}

void RsmProtocol::Send()
{
  Ptr< Node > sender = GetNode();
  uint32_t nid = sender->GetId();
  NS_LOG_DEBUG ("@" << Simulator::Now().GetSeconds() << " [RsmProtocol::SendBeacon] nid=" << nid);

  // retrieve state info
  //Ptr<VehicleMobilityModel> mm = sender->GetObject<VehicleMobilityModel>();
  //Ptr<ContextMap> cm = sender->GetObject<ContextMap>();
  //update self state
  m_contextMap->UpdateHostState(Simulator::Now());

  Ptr<Packet> packet = Create<Packet> (0);
  uint8_t bsmCount = AddDsrcBsmHeaders(packet);
  RsmpHeader rsmpHeader;
  rsmpHeader.SetBsmCount(bsmCount);
  packet->AddHeader(rsmpHeader);
  EnsureSize(packet);
  BroadcastPacket(packet, CalculateTxPowerLevel());
  
  //record actual interval
  Time actualInterval = Simulator::Now() - m_lastTransmission;
  m_lastTransmission=Simulator::Now();
  if (!s_sendCallback.IsNull()) s_sendCallback(nid, actualInterval, 0);

  Time interval = CalculateInterval();
  //std::cout << "interval=" << interval.GetSeconds() << std::endl;
  if (interval.IsStrictlyPositive() &&  s_stopTime.IsStrictlyPositive()) {
    if (Simulator::Now() + interval <= s_stopTime) {
      m_nextTransmission = Simulator::Schedule( interval, &RsmProtocol::Send, this );
    }
  }
  
}

void RsmProtocol::DoReceivePacket(Ptr< Node > node, Ptr< Packet > packet, const WsmpHeader& wsmpHeader)
{
  uint32_t receiverId = node->GetId();
  RsmpHeader rsmpHeader;
  packet->RemoveHeader(rsmpHeader);
  uint32_t senderId = wsmpHeader.GetSenderId();
  Time delay = Simulator::Now() - wsmpHeader.GetTimeStamp();
  //NS_LOG_DEBUG ("@"<<Simulator::Now()<<" ReceiveBeacon: Node=" << recvId << ", senderId=" << senderId << ", time="<< timestamp << ", delay="<< delay);
  //mdata->UpdateBeaconReceived(senderId, recvId, delay, 0);
  std::map<uint32_t,StateItem> states;
  RemoveDsrcBsmHeaders(packet, wsmpHeader, rsmpHeader.GetBsmCount(), states);
  std::map<uint32_t,StateItem>::const_iterator it;
  for (it=states.begin(); it != states.end(); it++ ) {
    m_contextMap->UpdateState (it->first, it->second);
  }
  //m_contextMap->SetRsmDelay(senderId, delay);
  Receive(node, packet, wsmpHeader, states);
  if (!s_recvCallback.IsNull()) s_recvCallback(senderId, receiverId, delay, 0);
}

void RsmProtocol::Initialize(Ptr< Node > node)
{
  m_contextMap = GetNode()->GetObject<ContextMap>();
  NS_ASSERT (m_contextMap != 0);
  InitiateSending();
}

void RsmProtocol::Receive(Ptr< Node > receiver, Ptr< Packet > packet, const WsmpHeader& wsmpHeader, const std::map< uint32_t, StateItem >& states)
{ }

uint8_t RsmProtocol::AddDsrcBsmHeaders(Ptr< Packet > packet)
{ //add one header
  StateItem si;
  m_contextMap->GetHostState(si);
  DsrcBsmHeader bsmHeader;
  bsmHeader.SetVehicleState(si.state);
  //bsmHeader.SetSumDL(su1mDL);
  bsmHeader.SetOffset(si.offset);
  bsmHeader.SetLaneId(si.laneId);
  packet->AddHeader(bsmHeader);
  return 1;
}

void RsmProtocol::RemoveDsrcBsmHeaders(Ptr< Packet > packet, const WsmpHeader& wsmpHeader, uint32_t n, std::map< uint32_t, StateItem >& states)
{ //remove one header
  DsrcBsmHeader bsmHeader;
  packet->RemoveHeader(bsmHeader);
  StateItem& si = states[wsmpHeader.GetSenderId()];
  si.state = bsmHeader.GetVehicleState();
  si.timestamp = wsmpHeader.GetTimeStamp();
  si.offset = bsmHeader.GetOffset();
  si.laneId = bsmHeader.GetLaneId();
  si.rsmDelay = Simulator::Now() - wsmpHeader.GetTimeStamp();
}

Time RsmProtocol::CalculateInterval() {
  return Seconds(0.0f);
}

Time RsmProtocol::CalculateStartTime() {
  return MilliSeconds(-1);
}

int RsmProtocol::CalculateTxPowerLevel()
{
  return -1;
}

void RsmProtocol::EnsureSize(Ptr< Packet > packet) const
{
  if (s_constantSize > 0) {
    uint32_t minSize = s_constantSize - WsmpHeader::SIZE;
    uint32_t fillSize = minSize - packet->GetSize();
    if (fillSize > 0) {
      packet->AddPaddingAtEnd(fillSize);
    }
    NS_ASSERT (packet->GetSize() == minSize);
  }
}





