#ifndef RSMP_CONSTANT_RATE_H
#define RSMP_CONSTANT_RATE_H

#include "ns3/random-variable.h"
#include "rsm-protocol.h"
#include "rsmp-header.h"

/**
 * constant size, rate, power. Random or sequential start time. Random jitter
 */
class RsmpConstantRate : public RsmProtocol {
  public:
    static TypeId GetTypeId(void);
    
    RsmpConstantRate();
    virtual ~RsmpConstantRate();
  
  protected:
    virtual Time CalculateStartTime();
    virtual Time CalculateInterval();

  private:
    UniformVariable m_random;
    /** Minimum interval between beacon transmissions. */
    static Time s_interval;
    static Time s_jitterInterval;
    /** Default is use random */
    static bool s_isRandomStart;
    
};

#endif // RSMP_CONSTANT_RATE_H
