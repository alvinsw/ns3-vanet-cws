/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/assert.h"
#include "ns3/log.h"

#include "esmp-header.h"

using namespace ns3;

NS_OBJECT_ENSURE_REGISTERED (EsmpHeader);
NS_LOG_COMPONENT_DEFINE ("EsmpHeader");

EsmpHeader::EsmpHeader() : originId(0), eventId(0), m_originTimestamp(), m_receivers() {
  //NS_ASSERT( sizeof(senderId) + sizeof(originId) + sizeof(eventId) + sizeof(m_originTimestamp.GetNanoSeconds()) + sizeof(m_timestamp.GetNanoSeconds()) == SIZE );
}

EsmpHeader::~EsmpHeader() { }

TypeId EsmpHeader::GetTypeId() {
  static TypeId tid = TypeId ("EsmpHeader")
    .SetParent<Header> ()
    .AddConstructor<EsmpHeader> ()
    ;
  return tid;
}

TypeId EsmpHeader::GetInstanceTypeId() const {
  return GetTypeId ();
}

void EsmpHeader::Print(std::ostream& os) const {
  os << "originId=" << originId << std::endl;
  os << "eventId=" << eventId << std::endl;
  os << "originTimestamp=" << m_originTimestamp << std::endl;
  os << "receivers(" << m_receivers.size() << ")=[";
  for (TReceivers::iterator it = m_receivers.begin(); it != m_receivers.end(); it++) {
    os << *it << ", ";
  }  
  os << "]" << std::endl;
}

uint32_t EsmpHeader::GetSerializedSize() const {
  return SIZE + (4 * m_receivers.size());
}

void EsmpHeader::Serialize(Buffer::Iterator start) const {
  start.WriteHtonU32(originId);
  start.WriteHtonU32(eventId);
  start.WriteHtonU64(m_originTimestamp.GetNanoSeconds());
  start.WriteHtonU32( m_receivers.size() );
  for (TReceivers::iterator it = m_receivers.begin(); it != m_receivers.end(); it++) {
    start.WriteHtonU32(*it);
  }
}

uint32_t EsmpHeader::Deserialize(Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them in host byte order.
  originId = start.ReadNtohU32();
  eventId = start.ReadNtohU32();
  m_originTimestamp = NanoSeconds( start.ReadNtohU64() );
  uint32_t receiversCount = start.ReadNtohU32();
  for (uint32_t i = 0; i < receiversCount; ++i) {
    m_receivers.insert( start.ReadNtohU32() );
  }
  // we return the number of bytes effectively read.
  return GetSerializedSize();
  //return SIZE;
}

void EsmpHeader::SetOriginId(uint32_t id) {
  originId = id;
}

uint32_t EsmpHeader::GetOriginId(void ) const {
  return originId;
}

void EsmpHeader::SetEventId(uint32_t id) {
  eventId = id;
}

uint32_t EsmpHeader::GetEventId() const {
  return eventId;
}

void EsmpHeader::SetOriginTimestamp(Time time) {
  m_originTimestamp = time;
}

Time EsmpHeader::GetOriginTimeStamp() const {
  return m_originTimestamp;
}

void EsmpHeader::AddReceiver(uint32_t r) {
  m_receivers.insert(r);
}

bool EsmpHeader::ContainsReceiver(uint32_t r) const {
  return m_receivers.find(r) != m_receivers.end();
}

std::set<uint32_t>& EsmpHeader::GetReceivers() {
  return m_receivers;
}

const std::set<uint32_t>& EsmpHeader::GetReceivers() const {
  return m_receivers;
}

