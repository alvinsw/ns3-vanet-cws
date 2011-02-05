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

#ifndef WSM_PROTOCOL_H
#define WSM_PROTOCOL_H

#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/wifi-net-device.h"

//#include "../road-traffic-scenario.h"
//#include "cws-measurement.h"
#include <string>
#include "wsmp-header.h"

using namespace ns3;

class WsmProtocol : public Object {
  public:
    static TypeId GetTypeId(void);
    //static std::string GetName();
    //static void Install(NodeContainer& nodes);
    static const uint16_t WSMP_NUMBER = 0x88DC; //0x88DC default protocol number
    static const uint16_t ESMP_NUMBER = 0x88DD;
    static const uint16_t RSMP_NUMBER = 0x88DE;
    //static double defaultTxPower;
    
    //static const uint8_t PT_WARNING = 1; //Protocol Type
    //static const uint8_t PT_BEACON = 2;
    //static const uint8_t PT_DUMMY = 3;
    //static const uint8_t PT_ACK = 4;
    static const uint8_t ESMP_PRIORITY = 6;
    static const uint8_t RSMP_PRIORITY = 5;//4
    static const uint8_t DUMMY_PRIORITY = 1;
    
    WsmProtocol();
    WsmProtocol(uint16_t protocolNumber, uint8_t priority);
    virtual ~WsmProtocol();
    
    //static void Install(NodeContainer& nodes, CwsMeasurement* mdata_, RoadTrafficScenario* rts_);
    //uint16_t GetType() const;
    void Install(Ptr<Node> node);
    void Deactivate();
    
    uint16_t GetProtocolNumber() const;
    Ptr<Node> GetNode() const;
    int GetMaxTxPowerLevel() const;
    
    /** Call this method to broadcast a packet */
    void BroadcastPacket (Ptr<Packet> packet, int powerLevel = -1);
    
    /** 
     * This method will be called when a packet is received by a device at a node. 
     * The packet then will be forwarded to a virtual method implemented in subclasses 
     * for different the protocol types.
     */
    void ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address& from, const Address& to, NetDevice::PacketType packetType);
   
    
    //double GetTxPower();
    //uint8_t GetTxPowerLevel();
    //void SetTxPowerLevel(uint8_t txPowerLevel);
       
  protected:
    /** This method will be called by Install() method */
    virtual void Initialize(Ptr<Node> node);
    
    /** Override this method to handle received packet for a specific protocol */
    virtual void DoReceivePacket (Ptr<Node> node, Ptr<Packet> packet, const WsmpHeader& wsmpHeader);
    
    //CwsMeasurement* mdata;
    //RoadTrafficScenario* rts;
  private:
    //std::string m_protocolName;
    uint16_t m_protocolNumber;
    uint8_t m_priority;
    uint8_t m_maxTxPowerLevel;
    //double m_txPower;
    Ptr<Node> m_node;
    Ptr<WifiNetDevice> m_wifiNetDevice;

};

#endif // PROTOCOL_H

