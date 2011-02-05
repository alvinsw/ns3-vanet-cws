/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ESMP_HEADER_H
#define ESMP_HEADER_H

#include "ns3/header.h"
#include "ns3/nstime.h"
#include <set>

using namespace ns3;

/* A Collision Warning Message Header implementation
 */
class EsmpHeader : public Header
{
public:
  typedef std::set<uint32_t> TReceivers;
  
  EsmpHeader();
  virtual ~EsmpHeader();
  
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
  
  /** The node that sends the packet first time  */
  void SetOriginId (uint32_t id);
  
  /** The node that sends the packet first time  */
  uint32_t GetOriginId (void) const;

  /** The time the msg created at the first sender */
  void SetOriginTimestamp(Time time);
  
  /** The time the msg created at the first sender */
  Time GetOriginTimeStamp() const;
  
  void SetEventId(uint32_t id);
  uint32_t GetEventId() const;
    
  void AddReceiver(uint32_t r);
  bool ContainsReceiver(uint32_t r) const;
  TReceivers& GetReceivers();
  const TReceivers& GetReceivers() const;

private:
  uint32_t originId; //4 - The first vehicle that send the msg
  uint32_t eventId; //4
  Time m_originTimestamp; //8
  TReceivers m_receivers;//4 - for the count
  
  static const uint16_t SIZE = 4+4+8+4;
};

#endif /* ESMP_HEADER_H */
