#include "experiment.h"
#include "../highway-scenario.h"
#include "ns3/core-module.h"

//static ns3::GlobalValue g_phyDataRate ("PhyDataRate", "The global data rate in Mbps.",
//                                       UintegerValue (6), MakeUintegerChecker<uint32_t> ());

using namespace ns3;

class ExpTest : public Experiment {
public:
  static TypeId GetTypeId(void);
  
  ExpTest();
  virtual ~ExpTest();
  virtual Ptr< RoadTrafficScenario > CreateRTS() const;
private:
};

NS_OBJECT_ENSURE_REGISTERED (ExpTest);
NS_LOG_COMPONENT_DEFINE ("ExpTest");

TypeId ExpTest::GetTypeId (void)
{
  static TypeId tid = TypeId("ExpTest")
    .SetParent<Object> ()
    .AddConstructor<ExpTest> ()
    ;
  return tid;
}

ExpTest::ExpTest() { 
  LogComponentEnable("RoadTrafficScenario", LOG_LEVEL_ALL);
}

ExpTest::~ExpTest() { }
//     viz(false), simDuration(NanoSeconds(0)), wpm(wpm_), rts(rts_), mdata(0),
//     measurementStart(Seconds(1)), measurementEnd(Seconds(2))

Ptr< RoadTrafficScenario > ExpTest::CreateRTS() const
{
  Ptr<VehicleType> aCar = Create<VehicleType>(4,2,2,"Car");
  SimpleLane lane1(10, 20.0f, 20.0f);
  Ptr<HighwayScenario> hs = Create<HighwayScenario>(500, 5, aCar);
  hs->AddHighwayLane(lane1, HighwayScenario::RIGHT);
  return hs;
  //return 0;
}
