/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/assert.h"
#include "ns3/log.h"

#include "wsmp-header.h"

using namespace ns3;

NS_OBJECT_ENSURE_REGISTERED (WsmpHeader);
NS_LOG_COMPONENT_DEFINE ("WsmpHeader");

/* A sample Header implementation
 */
WsmpHeader::WsmpHeader () : m_senderId(0), m_timestamp(MilliSeconds(0))
{}
WsmpHeader::~WsmpHeader () {}

TypeId 
WsmpHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("WsmpHeader")
    .SetParent<Header> ()
    .AddConstructor<WsmpHeader> ()
    ;
  return tid;
}

TypeId 
WsmpHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void 
WsmpHeader::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  os << "senderId=" << m_senderId << std::endl;
  os << "timestamp=" << m_timestamp << std::endl;
}

uint32_t
WsmpHeader::GetSerializedSize (void) const
{
  //NS_ASSERT( sizeof(senderId) + sizeof(m_timestamp.GetNanoSeconds()) == SIZE );
  //std::cout <<"GetSerializedSize="<< sizeof(senderId) + sizeof(m_timestamp.GetNanoSeconds()) << std::endl;
  return SIZE;
}

void
WsmpHeader::Serialize (Buffer::Iterator start) const
{
  // we can serialize two bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteHtonU32(m_senderId);
  start.WriteHtonU64(m_timestamp.GetNanoSeconds());
}

uint32_t
WsmpHeader::Deserialize (Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  m_senderId = start.ReadNtohU32();
  m_timestamp = NanoSeconds( start.ReadNtohU64() );
  // we return the number of bytes effectively read.
  return GetSerializedSize();
}

void 
WsmpHeader::SetSenderId (uint32_t id)
{
  m_senderId = id;
}

uint32_t 
WsmpHeader::GetSenderId (void) const
{
  return m_senderId;

}

ns3::Time 
WsmpHeader::GetTimeStamp () const
{
  return m_timestamp;
}

void 
WsmpHeader::SetTimestamp(Time time)
{
  m_timestamp = time;
}

