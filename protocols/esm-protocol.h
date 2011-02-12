#ifndef ESM_PROTOCOL_H
#define ESM_PROTOCOL_H

#include "ns3/simulator.h"
#include <map>
#include "esmp-header.h"
#include "dsrc-bsm-header.h"
#include "wsm-protocol.h"

  
/** Represents unique warning message for every emergency event */
struct Esm : public SimpleRefCount<Esm> {
  static const uint32_t MAX_COUNT = 10000;
  static uint32_t CreateMessageId(uint32_t origin, uint32_t event);
  
  Esm (uint32_t origin = 0, uint32_t event = 0, Time originTime = MilliSeconds(0));
  virtual ~Esm();
  uint32_t GetMessageId() const;
  virtual void DoDispose();
  
  uint32_t originId;
  uint32_t eventId;
  uint32_t sentMessages;
  Time originTimestamp;
  EventId scheduledEvent;
  //bool keepBroadcasting; //should a node keep sending this message?
};


/** Event-based Safety Message (ESM) or Emergency Warning Message (EWM) Dissemination Protocol.
  * Base class for all warning protocols. This is a 'do nothing' protocol 
  */
class EsmProtocol : public WsmProtocol {
  public:
    /**  uint32_t receiverId, uint32_t messageId, Time delay */
    typedef Callback< void, uint32_t, uint32_t, Time > RecvCallback;
    /**  uint32_t receiverId, uint32_t messageId */
    typedef Callback< void, uint32_t, uint32_t> SendCallback;
    
    static TypeId GetTypeId(void);
    static void SetRecvCallback(RecvCallback cb);
    static void SetSendCallback(SendCallback cb);
    static void SetMessageSize(uint32_t size);
    
    EsmProtocol();
    virtual ~EsmProtocol();
    
    virtual int GetTxPowerLevel() const;
    
    ////** Use this method to set the estimated delay value for this node */
    //virtual void ReceiveBeacon(uint32_t senderId, uint32_t receiverId, Time delay);
    
    /** This method is invoked to make this protocol start sending warning messages */
    virtual void SendInitialWarning();
    
  protected:
    virtual void DoDispose(void );    
    /** Get msg history with current event id */
    Ptr<Esm> GetLastEvent(); 
    Ptr<Esm> CreateNewEvent();
    Ptr<Esm> GetHistory(uint32_t messageId); 
    Ptr<Esm> GetHistory(uint32_t originId, uint32_t eventId); 
    Ptr<Esm> AddHistory(uint32_t originId, uint32_t eventId); 
    void AddHistory(Ptr<Esm> esm); 
    
    virtual Ptr<Esm> CreateEsm(uint32_t originId, uint32_t eventId, Time originTime);
    virtual void DoReceivePacket (Ptr<Node> node, Ptr<Packet> packet, const WsmpHeader& wsmpHeader);
    virtual void AddReceivers(Ptr<Esm> esm, EsmpHeader& esmpHeader);
    
    //** This method calls RecvCallback that should handle the statistic update, and also make the vehicle brake. */
    //void NotifyReceived(uint32_t messageId, Time delayFromOrigin);
    
    /** Do NOT use the packet pointer after calling this method!!! */
    virtual void SendEsm(Ptr<Esm> esm);
    /** 
     * Override this method to process warning. Warning header has been removed from the packet.
     * Return true will result in calling the RecvCallback afterwards.
     */
    virtual bool ReceiveEsm(Ptr<Node> receiver, Ptr<Packet> packet, const WsmpHeader& wsmpHeader, const EsmpHeader& esmpHeader, const DsrcBsmHeader& bsmHeader);
    

  typedef std::map<uint32_t, Ptr<Esm> > THistory;
    /** Warning message packet size in bytes */
    static uint16_t s_messageSize;
    static RecvCallback s_recvCallback;
    static SendCallback s_sendCallback;
private:
  uint32_t m_currentEventId;
  THistory m_history;
  
};


  
/** Warning message will be received by all relevant vehicles without any delay */
class EsmpInstantaneous : public EsmProtocol {
  public:
    static TypeId GetTypeId(void);
    EsmpInstantaneous();
    virtual ~EsmpInstantaneous();
    
    virtual void SendInitialWarning();
};


#endif // ESM_PROTOCOL_H
