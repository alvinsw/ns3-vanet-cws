/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef RSMP_HEADER_H
#define RSMP_HEADER_H

#include "ns3/header.h"
#include "ns3/nstime.h"

using namespace ns3;

/* 
 * Basic Routine Safety Message protocol Header implementation
 */
class RsmpHeader : public Header
{
  public:
    RsmpHeader ();
    virtual ~RsmpHeader ();
    
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual void Print (std::ostream &os) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start);
    virtual uint32_t GetSerializedSize (void) const;
    
    void SetBsmCount(uint8_t n);
    uint32_t GetBsmCount() const;
  
  private:
    uint8_t m_bsmCount; //1
    
    static const uint16_t SIZE = 1;
};

#endif /* RSMP_HEADER_H */
