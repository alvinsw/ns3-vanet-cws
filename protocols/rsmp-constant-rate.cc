/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/global-value.h"
#include "ns3/boolean.h"
#include "ns3/node-list.h"
#include "dsrc-bsm-header.h"
#include "rsmp-constant-rate.h"
//#include "context-map.h"


NS_LOG_COMPONENT_DEFINE ("RsmpConstantRate");
NS_OBJECT_ENSURE_REGISTERED (RsmpConstantRate);

Time RsmpConstantRate::s_interval;
Time RsmpConstantRate::s_jitterInterval;
bool RsmpConstantRate::s_isRandomStart;

TypeId 
RsmpConstantRate::GetTypeId(void ) {
  static TypeId tid = TypeId("ns3::RsmpConstantRate")
    .SetParent<RsmProtocol> ()
    .AddConstructor<RsmpConstantRate> () 
    ;
  return tid;
}

RsmpConstantRate::RsmpConstantRate() : m_random() 
{
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    TimeValue tv;
    GlobalValue::GetValueByName ("RsmInterval",tv);
    s_interval = tv.Get ();
    GlobalValue::GetValueByName ("RsmJitterInterval",tv);
    s_jitterInterval = tv.Get ();
    BooleanValue bv;
    GlobalValue::GetValueByName ("RsmRandomStart",bv);
    s_isRandomStart = bv.Get ();
  }
}
RsmpConstantRate::~RsmpConstantRate() {}

Time RsmpConstantRate::CalculateInterval() {
  uint32_t maxJitter = s_jitterInterval.GetMicroSeconds();
  if (maxJitter == 0) return s_interval;
  uint64_t baseInterval = s_interval.GetMicroSeconds() - (maxJitter/2);
  uint32_t jitter = m_random.GetInteger(0, maxJitter);
  return MicroSeconds(baseInterval+jitter);
}

Time RsmpConstantRate::CalculateStartTime() {
  if (s_isRandomStart) {
    //random
    return MicroSeconds( m_random.GetInteger(1, s_interval.GetMicroSeconds()-1) );
  } else {
    //sequential
    int64_t timeInNano = (GetNode()->GetId() * s_interval.GetNanoSeconds()) / NodeList::GetNNodes();
    return NanoSeconds(timeInNano);
  }
}

