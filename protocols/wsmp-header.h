/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef WSMP_HEADER_H
#define WSMP_HEADER_H

#include "ns3/header.h"
#include "ns3/nstime.h"

using namespace ns3;

/* A Collision Warning Message Header implementation
 */
class WsmpHeader : public Header
{
public:
  WsmpHeader ();
  virtual ~WsmpHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
  
  static const uint16_t SIZE = 4+8;
  
  /** The node that sends the packet */
  void SetSenderId (uint32_t id);
  
  /** The node that sends the packet */
  uint32_t GetSenderId (void) const;
  
  /** The time when this message is generated and sent to MAC layer. */
  void SetTimestamp (Time time);
  
  /** The time when this message is generated and sent to MAC layer. */
  Time GetTimeStamp () const;
  
private:
  uint32_t m_senderId; //4
  Time m_timestamp; //8
};

#endif /* WSMP_HEADER */
