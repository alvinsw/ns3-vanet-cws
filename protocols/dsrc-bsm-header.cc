/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/assert.h"
#include "ns3/log.h"

#include "dsrc-bsm-header.h"

using namespace ns3;

NS_OBJECT_ENSURE_REGISTERED (DsrcBsmHeader);
NS_LOG_COMPONENT_DEFINE ("DsrcBsmHeader");

DsrcBsmHeader::DsrcBsmHeader () : m_state(), m_msgCount(0), m_offset(0), m_laneId(0)//, m_nodeId(0)
{ }

DsrcBsmHeader::~DsrcBsmHeader () {}

TypeId 
DsrcBsmHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("DsrcBsmHeader")
    .SetParent<Header> ()
    .AddConstructor<DsrcBsmHeader> ()
    ;
  return tid;
}
TypeId 
DsrcBsmHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void 
DsrcBsmHeader::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  os << "state=" << m_state << std::endl;
  os << "msgCount=" << (int)m_msgCount << std::endl;
  os << "GetOffset=" << GetOffset() << std::endl;
  os << "GetLaneId=" << GetLaneId() << std::endl;
  //os << "GetNodeId=" << GetNodeId() << std::endl;
}

uint32_t
DsrcBsmHeader::GetSerializedSize (void) const
{
  return SIZE;
}

void
DsrcBsmHeader::Serialize (Buffer::Iterator start) const
{
  // we can serialize two bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteU8(m_msgCount);
  start.WriteHtonU16(m_state.acceleration.acceleration);
  start.WriteHtonU16(m_state.heading.heading);
  start.WriteHtonU32(m_state.position.x);
  start.WriteHtonU32(m_state.position.y);
  start.WriteHtonU16(m_state.size.length);
  start.WriteHtonU16(m_state.size.width);
  start.WriteHtonU16(m_state.speed.speed);
  start.WriteHtonU32(m_offset);
  start.WriteHtonU32(m_laneId);
  //start.WriteHtonU32(m_nodeId);
}

uint32_t
DsrcBsmHeader::Deserialize (Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  m_msgCount = start.ReadU8();
  m_state.acceleration.acceleration = start.ReadNtohU16();
  m_state.heading.heading = start.ReadNtohU16();
  m_state.position.x = start.ReadNtohU32();
  m_state.position.y = start.ReadNtohU32();
  m_state.size.length = start.ReadNtohU16();
  m_state.size.width = start.ReadNtohU16();
  m_state.speed.speed = start.ReadNtohU16();
  m_offset = start.ReadNtohU32();
  m_laneId = start.ReadNtohU32();
  //m_nodeId = start.ReadNtohU32();

  return GetSerializedSize();
}

void DsrcBsmHeader::SetMsgCount(uint8_t n) {
  m_msgCount = n;
}

uint32_t DsrcBsmHeader::GetMsgCount() const {
  return m_msgCount;
}

void DsrcBsmHeader::SetVehicleState(const sae::VehicleState& state) {
  m_state = state;
}

const sae::VehicleState& DsrcBsmHeader::GetVehicleState() const {
  return m_state;
}

uint32_t DsrcBsmHeader::GetLaneId() const {
  return m_laneId;
}

void DsrcBsmHeader::SetLaneId(uint32_t id) {
  m_laneId = id;
}

double DsrcBsmHeader::GetOffset() const {
  return (double)m_offset / 100.0f;
}

void DsrcBsmHeader::SetOffset(double meters) {
  m_offset = uint32_t(meters * 100.0f);
}

// uint32_t DsrcBsmHeader::GetNodeId() const
// {
//   return m_nodeId;
// }
// void DsrcBsmHeader::SetNodeId(uint32_t id)
// {
//   m_nodeId = id;
// }

