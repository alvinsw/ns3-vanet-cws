#ifndef RSM_PROTOCOL_H
#define RSM_PROTOCOL_H

#include "ns3/event-id.h"
#include "wsm-protocol.h"
#include "rsmp-header.h"
#include "../context-map.h"

/**
 * Do nothing RSM protocol. This protocol does not send any messages.
 */
class RsmProtocol : public WsmProtocol {
  public:
    /** uint32_t senderId, uint32_t receiverId, Time delay, uint32_t msgId */
    typedef Callback< void, uint32_t, uint32_t, Time, uint32_t > RecvCallback;
    /** uint32_t senderId, Time interval, uint32_t msgId */
    typedef Callback< void, uint32_t, Time, uint32_t > SendCallback;
    
    static TypeId GetTypeId(void);
    static void SetRecvCallback(RecvCallback cb);
    static void SetSendCallback(SendCallback cb);
    /** Start time relative to current simulation time */
    static void Start(Time time);
    /** Stop time relative to current simulation time */
    static void Stop(Time time);
    
    RsmProtocol();
    virtual ~RsmProtocol();

    //void SetNotifyPacketReceived(PacketReceivedCallback callback);
    //void SetInterval(Time interval);
    //void SetSize(uint32_t size);
    
    //virtual void SetInterval(Time min, Time max);
    //virtual void SetSize(uint32_t min, uint32_t max);
    //virtual uint32_t GetMaxSize() const;
    //virtual Time GetMinInterval() const;
    
  protected:
    //PacketReceivedCallback notifyBeaconReceived;
    void Send();
    /** Schedule sending the first message based on the CalculateStartTime method. */
    void InitiateSending();
    /// Override WsmProtocol methods
    virtual void Initialize(Ptr< Node > node);
    virtual void DoReceivePacket(Ptr< Node > node, Ptr< Packet > packet, const WsmpHeader& wsmpHeader);
    
    /** Override this method to process beacon. All RSM header has been removed from the packet */
    virtual void Receive(Ptr<Node> receiver, Ptr<Packet> packet, const WsmpHeader& wsmpHeader, const std::map<uint32_t,StateItem>& states);
    
    virtual uint8_t AddDsrcBsmHeaders(Ptr<Packet> packet);
    virtual void RemoveDsrcBsmHeaders(Ptr<Packet> packet, const WsmpHeader& wsmpHeader, uint32_t n, std::map<uint32_t,StateItem>& states);
    /** Return negative number for not to send */
    virtual Time CalculateStartTime();
    virtual Time CalculateInterval();
    virtual int CalculateTxPowerLevel();
    /** 
     * If RsmSize is > 0 then the rsm packet size will always be set to RsmSize (if not already greater).
     * Override this method to impose a different behaviour on packet size constraint.
     */
    virtual void EnsureSize(Ptr<Packet> packet) const;

    static Time s_startTime;
    static Time s_stopTime;
    static RecvCallback s_recvCallback;
    static SendCallback s_sendCallback;
    static uint32_t s_constantSize;
    
    Time m_lastTransmission;
    EventId m_nextTransmission;
    Ptr<ContextMap> m_contextMap;
    
};

#endif // RSM_PROTOCOL_H
