#include "highway-scenario.h"
#include "ns3/log.h"
#include "ns3/random-variable.h"

NS_LOG_COMPONENT_DEFINE ("HighwayScenario");
  
LaneInitialState::LaneInitialState(uint32_t vehiclesCount) : pVehiclesCount(vehiclesCount) { }

LaneInitialState::~LaneInitialState() { }

uint32_t LaneInitialState::VehiclesCount() const { return pVehiclesCount; }



HighwayScenario::HighwayScenario(double highwayLength, double laneWidth, Ptr<VehicleType> vehicleType) :
    m_highwayLength(highwayLength), m_HighwayLaneWidth(laneWidth), m_VehicleType(vehicleType), m_vehiclesCount(0), m_lanesDir()
    //m_laneIS(), //, m_lanePaths() 
{ }

HighwayScenario::~HighwayScenario() { }

uint32_t HighwayScenario::Setup() {
  NS_LOG_DEBUG("[HighwayScenario::Setup] Start");
  NS_ASSERT( m_vehiclesCount == ExpectedVehiclesCount() );
  NS_LOG_DEBUG("[HighwayScenario::Setup] Finish");
  return m_vehiclesCount;
}

void HighwayScenario::AddHighwayLane(const LaneInitialState& laneInit, Direction dir) {
  NS_ASSERT( m_VehicleType != 0 );
  uint32_t laneIndex = LanesCount();
  //m_laneIS.push_back(lane);
  //m_lanesDir.push_back(dir);
  double posY = laneIndex * m_HighwayLaneWidth;
  Vector pLeft(0, posY, 0);
  Vector pRight(m_highwayLength, posY, 0);
  Ptr<StraightLanePath> slpath;
  if (dir == RIGHT) {
    slpath = Create<StraightLanePath>(pLeft, pRight);
  } else {
    slpath = Create<StraightLanePath>(pRight, pLeft);
  }
  m_lanesDir.push_back(dir);
  Lane& lane = AddLane(slpath);
  NS_ASSERT( lane.initialStates.empty()==true );
  lane.initialStates.resize( laneInit.VehiclesCount(), VehicleState(0,0,0,m_VehicleType) );
  laneInit.SetupInitialState(lane.initialStates, *(lane.path));
  NS_ASSERT( lane.initialStates.size()==laneInit.VehiclesCount() );
  m_vehiclesCount += laneInit.VehiclesCount();
}

double HighwayScenario::GetHighwayLaneWidth() const { return m_HighwayLaneWidth; }
double HighwayScenario::GetHighwayLength() const { return m_highwayLength; }
Ptr< VehicleType > HighwayScenario::GetVehicleType() const { return m_VehicleType; }
uint32_t HighwayScenario::GetNextLaneId(uint32_t laneId, RoadTrafficScenario::Direction side) const
{
  uint32_t nextLaneId = -1;
  if (m_lanesDir[laneId] == side) {
    nextLaneId = laneId - 1;
  } else {
    nextLaneId = laneId + 1;
  }
  if (nextLaneId >= 0 && nextLaneId < m_lanesDir.size()) {
    if (m_lanesDir[nextLaneId] != m_lanesDir[laneId]) nextLaneId = -1;
  } else {
    nextLaneId = -1;
  }
  
  return nextLaneId;
}



NClusterLane::NClusterLane(uint32_t clusterCount, uint32_t vehiclePerCluster, 
                           float interVehicleSpacing, float interClusterSpacing, 
                           float vehicleSpeed, float vehicleAcceleration) : 
    LaneInitialState(clusterCount * vehiclePerCluster), pClustersCount(clusterCount), pVehiclesPerClusterCount(vehiclePerCluster),
    pInterVehicleSpacing(interVehicleSpacing), pInterClusterSpacing(interClusterSpacing),
    pVehicleSpeed(vehicleSpeed), pVehicleAcceleration(vehicleAcceleration) 
{ }

NClusterLane::~NClusterLane() { }

void NClusterLane::SetupInitialState(std::vector< RoadTrafficScenario::VehicleState >& laneState, LanePath& path) const {
  double offset = 0;
  uint32_t count = 0; //vehicle count in a cluster
  std::vector<RoadTrafficScenario::VehicleState>::reverse_iterator iter;
  for (iter=laneState.rbegin(); iter!=laneState.rend(); ++iter) {
    //iter->position = path.CalculatePoint(offset);
    iter->position = offset;
    iter->speed = pVehicleSpeed;
    iter->acceleration = pVehicleAcceleration;
    count++;
    if (count == pVehiclesPerClusterCount) {
      offset += pInterClusterSpacing;
      count = 0;
    } else {
      offset += pInterVehicleSpacing;
    }
    //NS_LOG_DEBUG(iter->position);
  }
}



SimpleLane::SimpleLane(uint32_t vehiclesCount, float interVehicleSpacing, float vehicleSpeed, float vehicleAcceleration) :
    LaneInitialState(vehiclesCount), pInterVehicleSpacing(interVehicleSpacing), pVehicleSpeed(vehicleSpeed), pVehicleAcceleration(vehicleAcceleration) 
{ }

SimpleLane::~SimpleLane() { }

void SimpleLane::SetupInitialState(std::vector< RoadTrafficScenario::VehicleState >& laneState, LanePath& path) const {
  double offset = 0;
  std::vector<RoadTrafficScenario::VehicleState>::reverse_iterator iter;
  for (iter=laneState.rbegin(); iter!=laneState.rend(); ++iter) {
    iter->position = offset;
    iter->speed = pVehicleSpeed;
    iter->acceleration = pVehicleAcceleration;
    offset += pInterVehicleSpacing;
    //NS_LOG_DEBUG(iter->position);
  }
}



RandomSpacingLane::RandomSpacingLane(uint32_t vehiclesCount, float minSpacing, float maxSpacing, float vehicleSpeed, float vehicleAcceleration) : 
    LaneInitialState(vehiclesCount), pMinSpacing(minSpacing), pMaxSpacing(maxSpacing), pVehicleSpeed(vehicleSpeed), pVehicleAcceleration(vehicleAcceleration)
{ }

RandomSpacingLane::~RandomSpacingLane() { }

void RandomSpacingLane::SetupInitialState(std::vector< RoadTrafficScenario::VehicleState >& laneState, LanePath& path) const {
  double offset = 0;
  std::vector<RoadTrafficScenario::VehicleState>::reverse_iterator iter;
  for (iter=laneState.rbegin(); iter!=laneState.rend(); ++iter) {
    iter->position = offset;
    iter->speed = pVehicleSpeed;
    iter->acceleration = pVehicleAcceleration;
    UniformVariable random;
    double s = random.GetValue(pMinSpacing, pMaxSpacing);
    offset += s;
  }

}



LinearSpacingAndSpeedLane::LinearSpacingAndSpeedLane(uint32_t vehiclesCount, float minSpacing, float maxSpacing, float minSpeed, float maxSpeed, float vehicleAcceleration) :
    LaneInitialState(vehiclesCount), pMinSpacing(minSpacing), pMaxSpacing(maxSpacing), pMinSpeed(minSpeed), pMaxSpeed(maxSpeed), pVehicleAcceleration(vehicleAcceleration)
{ }

LinearSpacingAndSpeedLane::~LinearSpacingAndSpeedLane() {}

void LinearSpacingAndSpeedLane::SetupInitialState(std::vector< RoadTrafficScenario::VehicleState >& laneState, LanePath& path) const {
  double spaceStep = (pMaxSpacing - pMinSpacing) / (pVehiclesCount-1);
  double speedStep = (pMaxSpeed - pMinSpeed) / (pVehiclesCount-1);
  double offset = 0;
  double space = pMaxSpacing;
  double speed = pMaxSpeed;
  std::vector<RoadTrafficScenario::VehicleState>::reverse_iterator iter;
  for (iter=laneState.rbegin(); iter!=laneState.rend(); ++iter) {
    iter->position = offset;
    iter->speed = speed;
    iter->acceleration = pVehicleAcceleration;
    offset += space;
    space -= spaceStep;
    speed -= speedStep;
  }  
}


