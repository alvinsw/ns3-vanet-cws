/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/assert.h"
#include "ns3/log.h"

#include "rsmp-header.h"

using namespace ns3;

NS_OBJECT_ENSURE_REGISTERED (RsmpHeader);
NS_LOG_COMPONENT_DEFINE ("RsmpHeader");

/* A sample Header implementation
 */
RsmpHeader::RsmpHeader () : m_bsmCount(0)
{ }

RsmpHeader::~RsmpHeader () {}

TypeId 
RsmpHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("RsmpHeader")
    .SetParent<Header> ()
    .AddConstructor<RsmpHeader> ()
    ;
  return tid;
}

TypeId 
RsmpHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void 
RsmpHeader::Print (std::ostream &os) const
{
  os << "bsmCount=" << (int)m_bsmCount << std::endl;
}

uint32_t
RsmpHeader::GetSerializedSize (void) const
{
  return SIZE;
}

void
RsmpHeader::Serialize (Buffer::Iterator start) const
{
  start.WriteU8(m_bsmCount);
}

uint32_t
RsmpHeader::Deserialize (Buffer::Iterator start)
{
  m_bsmCount = start.ReadU8();
  return GetSerializedSize();
}

uint32_t RsmpHeader::GetBsmCount() const
{
  return (uint32_t)m_bsmCount;
}

void RsmpHeader::SetBsmCount(uint8_t n)
{
  m_bsmCount = n;
}

