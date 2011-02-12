#ifndef ESMP_IBIA_H
#define ESMP_IBIA_H

#include "esm-protocol.h"
  
/** 
  * Intelligent Broadcast with Implicit Acknowledgement (IBIA) protocol.
  */
class EsmpIbia : public EsmProtocol {
  public:
    
    static TypeId GetTypeId(void);
    
    EsmpIbia();
    virtual ~EsmpIbia();
    
    /** This method is invoked to make this protocol start sending warning messages */
    virtual void SendInitialWarning();
    
  protected:
    virtual bool ReceiveEsm(Ptr<Node> receiver, Ptr<Packet> packet, const WsmpHeader& wsmpHeader, const EsmpHeader& esmpHeader, const DsrcBsmHeader& bsmHeader);
    void SendAndRepeat(Ptr<Esm> esm);
    void ScheduleSend(Time t, Ptr<Esm> esm);
    
  private:
    static Time s_repeatInterval;
    static Time s_maxRelayWait;
    static uint32_t s_repeatCount;
};


#endif // ESMP_IBIA_H
