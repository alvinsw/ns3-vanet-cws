/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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

#include "wsm-protocol.h"

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
//#include "ns3/packet.h"
#include "ns3/qos-tag.h"
#include "ns3/txpowerlevel-tag.h"
#include "ns3/wifi-phy.h"
//#include "ns3/config.h"
//#include "ns3/double.h"

using namespace ns3;

NS_OBJECT_ENSURE_REGISTERED (WsmProtocol);
NS_LOG_COMPONENT_DEFINE ("WsmProtocol");

//double WsmProtocol::defaultTxPower = 0.0f;
//const uint16_t WsmProtocol::PROTOCOL_NUMBER = 0x88DC;

TypeId 
WsmProtocol::GetTypeId(void) {
  static TypeId tid = TypeId("WsmProtocol")
    .SetParent<Object> ()
    .AddConstructor<WsmProtocol> ()
    ;
  return tid;
}


WsmProtocol::WsmProtocol() : m_protocolNumber(WSMP_NUMBER), m_priority(0)
{ }

WsmProtocol::WsmProtocol(uint16_t protocolNumber, uint8_t priority): m_protocolNumber(protocolNumber), m_priority(priority)
{ }

WsmProtocol::~WsmProtocol() { }

void 
WsmProtocol::Install(Ptr< Node > node)
{
  node->AggregateObject (this);
  node->RegisterProtocolHandler(MakeCallback (&WsmProtocol::ReceivePacket, this), m_protocolNumber, 0); //install to all netdevice
  m_node = node;
  m_wifiNetDevice = node->GetDevice(0)->GetObject<WifiNetDevice>();
  Ptr<WifiPhy> phy = m_wifiNetDevice->GetPhy();
  m_maxTxPowerLevel = phy->GetNTxPower() - 1;
  Initialize(node);
}

void WsmProtocol::Deactivate()
{
  m_node->UnregisterProtocolHandler(MakeCallback (&WsmProtocol::ReceivePacket, this));
  m_node = 0;
}

uint16_t WsmProtocol::GetProtocolNumber() const
{
  return m_protocolNumber;
}

Ptr< Node > WsmProtocol::GetNode() const
{
  return m_node;
}

int WsmProtocol::GetMaxTxPowerLevel() const
{
  return m_maxTxPowerLevel;
}

void WsmProtocol::BroadcastPacket(Ptr< Packet > packet, int powerLevel) {
  NS_ASSERT (m_node != 0);
  NS_LOG_DEBUG("@" << Simulator::Now() << " [WsmProtocol::BroadcastPacket] NodeId=" << m_node->GetId() 
                   << " protocolNumber=" << m_protocolNumber << " priority=" << (uint32_t)m_priority 
                   << " powerLevel="<<powerLevel );
  packet->RemoveAllPacketTags();
  if (powerLevel >= 0 && powerLevel <= GetMaxTxPowerLevel()) {
    TxPowerLevelTag txplTag((uint8_t)powerLevel);
    packet->AddPacketTag(txplTag);
  }
  QosTag qosTag(m_priority);
  packet->AddPacketTag(qosTag);
  
  WsmpHeader header;
  header.SetSenderId(m_node->GetId());
  header.SetTimestamp(Simulator::Now());
  packet->AddHeader(header);
  
  m_wifiNetDevice->Send(packet, m_wifiNetDevice->GetBroadcast(), m_protocolNumber);
}

//Config::Set("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/TxPowerStart", DoubleValue(m_txPower));
//Config::Set("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/TxPowerEnd", DoubleValue(m_txPower));
void WsmProtocol::ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address& from, const Address& to, NetDevice::PacketType packetType) {
  NS_ASSERT (protocol == m_protocolNumber);
  NS_ASSERT (device->GetNode() == m_node);
  //NS_LOG_DEBUG ("@"<<Simulator::Now() << ": ReceivePacket - Node: " << device->GetNode()->GetId());
  Ptr<Packet> newPacket = packet->Copy();
  //newPacket->RemoveAllPacketTags();
  NS_ASSERT (packet->GetSize() == 500);
  WsmpHeader header;
  newPacket->RemoveHeader(header);
  DoReceivePacket(m_node, newPacket, header);
  //calculate received packet size
  //uint16_t rid = device->GetNode()->GetId();
  //mdata->nodes[rid].receivedPacketSize += packet->GetSize();
}

void WsmProtocol::DoReceivePacket(Ptr<Node> node, Ptr<Packet> packet, const WsmpHeader& wsmpHeader) {
  NS_LOG_DEBUG ("@"<<Simulator::Now() << " [WsmProtocol::DoReceivePacket]: Node=" << node->GetId() << " Packet=" << packet);
}

void WsmProtocol::Initialize(Ptr< Node > node)
{ }


