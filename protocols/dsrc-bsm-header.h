/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef DSRC_BSM_HEADER_H
#define DSRC_BSM_HEADER_H

#include "ns3/header.h"
#include "../sae-types.hpp"

using namespace ns3;

/* 
 * Basic safety message Header implementation.
 */
class DsrcBsmHeader : public Header
{
  public:
    DsrcBsmHeader ();
    virtual ~DsrcBsmHeader ();
    
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual void Print (std::ostream &os) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start);
    virtual uint32_t GetSerializedSize (void) const;
    
    void SetMsgCount(uint8_t n);
    uint32_t GetMsgCount() const;
    void SetVehicleState(const sae::VehicleState& state);
    const sae::VehicleState& GetVehicleState() const;
    void SetOffset(double meters);
    double GetOffset() const;
    void SetLaneId(uint32_t id);
    uint32_t GetLaneId() const;
//     void SetNodeId(uint32_t id);
//     uint32_t GetNodeId() const;
    //** Channel load is in number of packets per second. */
    //void SetChannelLoad(uint32_t load);
    //** Channel load is in number of packets per second. */
    //uint32_t GetChannelLoad() const;
  
  private:
    sae::VehicleState m_state;
    uint8_t m_msgCount; //1
    //uint32_t m_sumDL; //4 sum of danger level of local knowledge
    uint32_t m_offset; //4
    uint32_t m_laneId; //4
    //uint32_t m_nodeId; //4
    //uint32_t m_channelLoad; //4
    
    static const uint16_t SIZE = sae::VehicleState::SIZE + 1 + 4 + 4 ;
};

#endif /* DSRC_BSM_HEADER_H */
