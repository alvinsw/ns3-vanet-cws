/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "esmp-ibia.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/random-variable.h"

//#include "ns3/simulator.h"
#include "../default-parameters.h"
#include "../vehicle-mobility-model.h"

NS_OBJECT_ENSURE_REGISTERED (EsmpIbia);
NS_LOG_COMPONENT_DEFINE ("EsmpIbia");

Time EsmpIbia::s_repeatInterval = DP_ESM_INTERVAL;
Time EsmpIbia::s_maxRelayWait = DP_IBIA_MAX_WAIT;
uint32_t EsmpIbia::s_repeatCount = DP_IBIA_REPEAT_COUNT;


TypeId 
EsmpIbia::GetTypeId(void) {
  static TypeId tid = TypeId("EsmpIbia")
    .SetParent<EsmProtocol> ()
    .AddConstructor<EsmpIbia> () 
    ;
  return tid;
}

EsmpIbia::EsmpIbia() : EsmProtocol()
{ }

EsmpIbia::~EsmpIbia() 
{ }

void EsmpIbia::SendInitialWarning() {
  NS_ASSERT (GetNode() != 0);
  SendAndRepeat(CreateNewEvent());
}

bool 
EsmpIbia::ReceiveEsm(Ptr< Node > receiver, Ptr< Packet > packet, const WsmpHeader& wsmpHeader, const EsmpHeader& esmpHeader, const DsrcBsmHeader& bsmHeader)
{ 
  bool isReceivedFirstTime = false;
  uint32_t sid = wsmpHeader.GetSenderId();
  uint32_t rid = receiver->GetId();
  uint32_t originId = esmpHeader.GetOriginId();
  uint32_t eventId = esmpHeader.GetEventId();
  Ptr<VehicleMobilityModel> m = receiver->GetObject<VehicleMobilityModel>();
  NS_LOG_DEBUG ("@" << Simulator::Now() << " [EsmpIbia::ReceiveEsm] rid=" << rid << " sid=" << sid << " originTime=" << esmpHeader.GetOriginTimeStamp());
  
  if (bsmHeader.GetLaneId() == m->GetLaneId()) {
    Ptr<Esm> esm = GetHistory(originId, eventId);
    if (bsmHeader.GetOffset() > m->GetOffsetAlongPath()) { //message come from front
      if ( esm == 0 ) { //first time reception of message
        isReceivedFirstTime = true;
        // put to history
        esm = CreateEsm(originId, eventId, esmpHeader.GetOriginTimeStamp());
        AddHistory(esm);

        //wait for random duration
        UniformVariable random;
        uint32_t randomDelay = random.GetInteger(0, s_maxRelayWait.GetMicroSeconds());
        ScheduleSend(MicroSeconds(randomDelay), esm);
        NS_LOG_DEBUG ("@" << Simulator::Now() << " [EsmpIbia::ReceiveEsm] relay msg: id="<< rid << " delay(ms)=" << randomDelay << " senderId=" << sid);
      }
    } else { //message come from back
      if ( esm != 0 ) { // received the same message from behind
        esm->scheduledEvent.Cancel();
        //esm->scheduledEvent = EventId();
      }
    }
  }
  return isReceivedFirstTime; 
}

void EsmpIbia::SendAndRepeat(Ptr< Esm > esm)
{
  SendEsm(esm);
  if (esm->sentMessages < s_repeatCount) {
    ScheduleSend(s_repeatInterval, esm);
  }
}

void EsmpIbia::ScheduleSend(Time t, Ptr< Esm > esm)
{
  esm->scheduledEvent = Simulator::Schedule(t, &EsmpIbia::SendAndRepeat, this, PeekPointer(esm) );
}

