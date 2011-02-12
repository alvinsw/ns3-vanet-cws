#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/random-variable.h"

#include "road-traffic-scenario.h"
#include "driver-input.h"
#include "driver-model.h"
#include "lane-path.h"
#include <limits>
#include <stdexcept>
#include "default-parameters.h"

NS_LOG_COMPONENT_DEFINE ("RoadTrafficScenario");

//double RoadTrafficScenario::defaultDistanceEpsilon = 0.001f;
bool RoadTrafficScenario::defaultStopAtEndPoint = true;

RoadTrafficScenario::AccelChangeEvent::AccelChangeEvent ( Time time_, uint32_t laneId_, uint32_t indexAtLane_, float accel_ ) :
        time(time_), laneId(laneId_), indexAtLane(indexAtLane_), accel(accel_)
{}

RoadTrafficScenario::VehicleState::VehicleState ( double position_, float speed_, float acceleration_, Ptr< VehicleType > vehicleType_ ) :
        position(position_), speed(speed_), acceleration(acceleration_), vehicleType(vehicleType_)
{}

////////////////////// Lane
//RoadTrafficScenario::Lane::Lane() : laneId(0), path(0), initialStates() { }
RoadTrafficScenario::Lane::Lane(Ptr<LanePath> p) : path(p), initialStates() { }
RoadTrafficScenario::Lane::~Lane() {}


RoadTrafficScenario::NodeData::NodeData() : projectedState(), projectedTime(MilliSeconds(-1)), isManual(false)
{ }

RoadTrafficScenario::RoadTrafficScenario() : 
    pAccelChangeEvents(),
    //pVehicleWidth(VEHICLE_W), pVehicleLength(VEHICLE_L), pVehicleSpeed(VEHICLE_SPEED), 
    pVehicleAbnormalDecel(DP_VEH_MAX_DECEL), pVehicleNormalDecel(DP_VEH_DECEL),
    pReactionTimeMin(DP_REACTION_TIME_MIN), pReactionTimeMax(DP_REACTION_TIME_MAX), pMinGap(DP_VEH_MIN_GAP), 
    pUpdatePeriod(DP_MOVEMENT_UPDATE_PERIOD), 
    m_notifyAccelerationChange(), m_notifyCollision(), m_notifyActiveStatusChange(), 
    m_lanes(), m_vehiclesCount(0), 
    m_stopSimulationAllowed(true), m_smartBrakingEnabled(true), m_stopAtEndPoint(defaultStopAtEndPoint),
    //m_smartBrakingPredictedStopOffset(), m_collisionDistance(), //m_isManual(), 
    m_driverInput(), m_driverModel(), m_data()
{ }

RoadTrafficScenario::~RoadTrafficScenario() { 
  //NS_LOG_DEBUG("~RoadTrafficScenario");
}

uint32_t RoadTrafficScenario::ExpectedVehiclesCount() const
{
  uint32_t count = 0;
  for (uint32_t i=0; i<m_lanes.size(); ++i) {
    count += m_lanes[i].initialStates.size();
  }
  return count;
}

uint32_t 
RoadTrafficScenario::VehiclesCount() const {
  return m_vehiclesCount;
}

void 
RoadTrafficScenario::AllowsStopSimulation(bool val) {
  m_stopSimulationAllowed = val;
}

void 
RoadTrafficScenario::EnablesSmartBraking(bool val) {
  m_smartBrakingEnabled = val;
}

void 
RoadTrafficScenario::SetNotifyAccelerationChange(RoadTrafficScenario::NotifyAccelerationChange callback) {
  m_notifyAccelerationChange = callback;
}

void 
RoadTrafficScenario::SetNotifyCollision(RoadTrafficScenario::NotifyCollision callback) {
  m_notifyCollision = callback;
}

void RoadTrafficScenario::SetNotifyActiveStatusChange(VehicleMobilityModel::ActiveStatusChangeCallback callback)
{
  m_notifyActiveStatusChange = callback;
}

void 
RoadTrafficScenario::SetReactionTime(Time min, Time max) {
  pReactionTimeMin = min;
  pReactionTimeMax = max;
}

void 
RoadTrafficScenario::SetReactionTime(Time rt) {
  SetReactionTime(rt, rt);
}

void 
RoadTrafficScenario::SetDriverInput ( Ptr< DriverInput > di )
{
  di->SetRTS(this);
  m_driverInput = di;
}

void 
RoadTrafficScenario::SetDriverModel ( Ptr< DriverModel > dm ) {
  dm->SetRTS(this);
  m_driverModel = dm;
}

void 
RoadTrafficScenario::PreSetAccident(Time accidentTime, uint32_t laneId, uint32_t indexAtLane) {
  PreSetAccelerationChange(accidentTime, laneId, indexAtLane, -pVehicleAbnormalDecel);
  //pAccidents.push_back( Accident(accidentTime, laneId, indexAtLane) );
}

void 
RoadTrafficScenario::PreSetAccelerationChange(Time t, uint32_t laneId, uint32_t indexAtLane, float a)
{
  pAccelChangeEvents.push_back( AccelChangeEvent(t, laneId, indexAtLane, a) );
}

RoadTrafficScenario::Lane& 
RoadTrafficScenario::AddLane(Ptr<LanePath> lanePath) {
  uint32_t laneId = LanesCount();
  lanePath->SetId(laneId);
  m_lanes.push_back(RoadTrafficScenario::Lane());
  Lane& lane = m_lanes.back();
  lane.path = lanePath;
  return lane;
}

RoadTrafficScenario::Lane& 
RoadTrafficScenario::GetLane(uint32_t laneId) {
  return m_lanes[laneId];
}

const RoadTrafficScenario::Lane& 
RoadTrafficScenario::GetLane(uint32_t laneId) const
{
  return m_lanes[laneId];
}

uint32_t 
RoadTrafficScenario::LanesCount() const {
  return m_lanes.size();
}

Time 
RoadTrafficScenario::CalculateReactionTime(uint32_t nodeId) const {
  if (pReactionTimeMin == pReactionTimeMax) {
    return pReactionTimeMin;
  } else { 
    //random reaction time
    UniformVariable random;
    uint32_t t_in_ms = random.GetInteger(pReactionTimeMin.GetMilliSeconds(), pReactionTimeMax.GetMilliSeconds());
    return MilliSeconds(t_in_ms);
  }
}

void 
RoadTrafficScenario::Install(NodeContainer& nodes) {
  NS_LOG_DEBUG ("[RoadTrafficScenario::Install] Start");
  NS_LOG_DEBUG ("[RoadTrafficScenario::Install] UpdatePeriod=" << pUpdatePeriod.GetMilliSeconds() <<"ms");
  NS_ASSERT (nodes.GetN() == 0);
  
  if (m_vehiclesCount <= 0) { //the scenario has not been set up
    m_vehiclesCount = Setup();
  }
  
  //NS_LOG_DEBUG ("m_vehiclesCount=" << m_vehiclesCount);
  //NS_LOG_DEBUG ("m_lanes.size()=" << m_lanes.size());
  nodes.Create(m_vehiclesCount);
  // init private data
  m_data.clear();
  m_data.resize(m_vehiclesCount);
  
  uint32_t nodeId = 0;
  uint32_t laneCount = m_lanes.size();
  for (uint32_t laneId=0; laneId<laneCount; ++laneId) {
    Lane& lane = m_lanes[laneId];
    nodeId = SetupLane(lane, nodeId, nodes);
  }
  
  // create accident which will initiate warning message
  // The specified vehicle node will perform braking at the specified time using maxDeceleration
  for (std::size_t i = 0; i < pAccelChangeEvents.size(); ++i) {
    AccelChangeEvent& ace = pAccelChangeEvents[i];
    uint32_t id = FindNodeId(ace.laneId, ace.indexAtLane);
    Ptr<Node> n = nodes.Get(id);
    //Simulator::Schedule(ace.time, &RoadTrafficScenario::ChangeAcceleration, this, n, ace.accel);
    m_data[id].isManual = true;
    ScheduleChangeAcceleration(n, ace.accel, ace.time);
  }
  
  Simulator::Schedule (pUpdatePeriod, &RoadTrafficScenario::Update, this);
  
  NS_LOG_DEBUG ("[RoadTrafficScenario::Install] Finish");
}

void RoadTrafficScenario::Dispose()
{
  for (uint32_t laneId=0; laneId<m_lanes.size(); ++laneId) {
    m_lanes[laneId].path->Reset();
    m_lanes[laneId].path = 0;
  }
  m_lanes.clear();
  //m_driverInput->SetHost(0);
  //m_driverInput->SetRTS(0);
  //m_driverModel->SetRTS(0);
  m_driverInput = 0;
  m_driverModel = 0;
}

Ptr< Node > RoadTrafficScenario::GetFollower(Ptr< Node > node) const {
  Ptr<VehicleMobilityModel> model = node->GetObject<VehicleMobilityModel>();
  Ptr<VehicleMobilityModel> mf = model->GetFollower();
  if (mf) return mf->GetNode();
  return 0;
}

Ptr< Node > RoadTrafficScenario::GetLeader(Ptr< Node > node) const {
  Ptr<VehicleMobilityModel> model = node->GetObject<VehicleMobilityModel>();
  Ptr<VehicleMobilityModel> ml = model->GetLeader();
  if (ml) return ml->GetNode();
  return 0;
}

void
RoadTrafficScenario::ScheduleEmergencyBraking(Ptr<Node> node, Time t) {
  ScheduleChangeAcceleration (node, -pVehicleAbnormalDecel, t);
  m_data[node->GetId()].isManual = true;
}

void 
RoadTrafficScenario::ScheduleNormalBraking(Ptr<Node> node, Time t) {
  ScheduleChangeAcceleration (node, -pVehicleNormalDecel, t);
  m_data[node->GetId()].isManual = true;
}

void 
RoadTrafficScenario::InitiateBraking(Ptr< Node > node) {
  if ( m_smartBrakingEnabled ) {
    InitiateSmartBraking(node);
  } else {
    ScheduleNormalBraking(node, CalculateReactionTime(node->GetId()));
  }
}

void
RoadTrafficScenario::InitiateSmartBraking(Ptr<Node> node) {
  uint32_t nid = node->GetId();
  Ptr<VehicleMobilityModel> m = node->GetObject<VehicleMobilityModel>();
  Ptr<VehicleMobilityModel> mfront;
  Ptr<Node> leader = GetLeader(node);
  if (leader == 0) {
    ScheduleNormalBraking(node, CalculateReactionTime(node->GetId()));    
  }
  double vlength = 0.0f;
  double minvlength = 0.0f;
  double minStopOffset = std::numeric_limits<double>::infinity();
  while (leader != 0) {
    RoadTrafficScenario::VehicleState lvs = m_data[leader->GetId()].projectedState;
    if (lvs.acceleration < 0) {
      double stopOffset = lvs.position - (lvs.speed * lvs.speed * 0.5f / lvs.acceleration) - vlength;
      //NS_LOG_DEBUG ("leader=" << leader->GetId() << "stopOffset="<<stopOffset);
      if (stopOffset < minStopOffset) {
        minStopOffset = stopOffset;
        minvlength = vlength;
        mfront = node->GetObject<VehicleMobilityModel>();
      }
    }
    vlength += (pMinGap + lvs.vehicleType->GetLength());
    leader = GetLeader(leader);
  }
  if (minStopOffset < std::numeric_limits<double>::infinity()) {
    //NS_LOG_DEBUG (" minStopOffset=" << minStopOffset << ", minvlength="<< minvlength);
    double cd = minStopOffset - (pMinGap + mfront->GetLength()/2 + m->GetLength()/2);
    //double cd = minStopOffset - (minvlength + pMinGap + mfront->GetLength()/2 + m->GetLength()/2);
    Time rt = CalculateReactionTime(nid);
    double v = m->GetSpeed();
    double reactionDistance = v * rt.GetSeconds();
    //double decel = 0.5f * v * v / (cd - reactionDistance);
    double decel = 0.5f * v * v / (cd - m->GetOffsetAlongPath() - reactionDistance);
    //if (decel < 0 || decel > pVehicleNormalDecel) decel = pVehicleNormalDecel;
    if (decel > pVehicleNormalDecel) decel = pVehicleNormalDecel;
    NS_LOG_DEBUG ("@" << Simulator::Now() << " [RoadTrafficScenario::InitiateSmartBraking] nodeId=" << nid << ", decel="<< decel << ", cd="<<cd <<" v="<<v<<" rt="<<rt );
    ScheduleChangeAcceleration(node, -decel, rt);
    m_data[node->GetId()].isManual = true;
  }
}

void RoadTrafficScenario::ChangeAcceleration(Ptr< Node > node, double newAcceleration)
{
  Ptr<VehicleMobilityModel> m = node->GetObject<VehicleMobilityModel>();
  NS_ASSERT (m != 0);
  double oldAccel = m->GetAcceleration();
  m->SetAcceleration(newAcceleration);
  NS_LOG_DEBUG ("@" << Simulator::Now() << " [RoadTrafficScenario::ChangeAcceleration] Node=" << node->GetId() << " a=" << newAcceleration <<"m/s^2");
  if (!m_notifyAccelerationChange.IsNull()) m_notifyAccelerationChange(node, oldAccel, newAcceleration);
}

void RoadTrafficScenario::ScheduleChangeAcceleration(Ptr< Node > node, double newAcceleration, Time t)
{
  NS_LOG_DEBUG ("@" << Simulator::Now() << " [RoadTrafficScenario::ScheduleChangeAcceleration] nodeId=" << node->GetId() << " a=" << newAcceleration << " t=" << t);
  //m_isManual[node->GetId()] = true;
  if (t.IsZero()) {
    ChangeAcceleration(node, newAcceleration);
  } else if (t.IsStrictlyPositive()) {
    uint32_t nid = node->GetId();
    RoadTrafficScenario::VehicleState& vs = m_data[nid].projectedState;
    double s, v0, v1, a, tsec;
    if (m_data[nid].projectedTime > Simulator::Now()) {
      tsec = (Simulator::Now() + t - m_data[nid].projectedTime).GetSeconds();
      NS_ASSERT (tsec >= 0); //cannot schedule accel change before the latest scheduled change
      s = vs.position;
      v0 = vs.speed;
      a = vs.acceleration;
    } else {
      tsec = t.GetSeconds();
      Ptr<VehicleMobilityModel> m = node->GetObject<VehicleMobilityModel>();
      NS_ASSERT (m!=0);
      s = m->GetOffsetAlongPath();
      v0 = m->GetSpeed();
      a = m->GetAcceleration();
    }
    v1 = v0 + a * tsec;
    if (v1 < 0) {
      v1 = 0;
      tsec = -v0 / a;
    }
    vs.position = s + v0*tsec + a*tsec*tsec*0.5;
    vs.speed = v1;
    vs.acceleration = newAcceleration;
    m_data[nid].projectedTime = Simulator::Now() + t;
    Simulator::Schedule(t, &RoadTrafficScenario::ChangeAcceleration, this, node, newAcceleration);
  }
}

void 
RoadTrafficScenario::Update() {
  //NS_LOG_DEBUG ("debug Update()");
  uint32_t activeCount = 0;
  for (uint32_t i=0; i<m_lanes.size(); ++i) {
    //NS_LOG_DEBUG ("[RoadTrafficScenario::Update] laneId="<< i);
    activeCount += UpdateLane(m_lanes[i]);
  }
  //NS_LOG_UNCOND("activeCount="<<activeCount);
  if (activeCount > 0) {
    Simulator::Schedule (pUpdatePeriod, &RoadTrafficScenario::Update, this);
  } else if (m_stopSimulationAllowed) {
    Simulator::Stop();
  }
}

uint32_t 
RoadTrafficScenario::UpdateLane(RoadTrafficScenario::Lane& lane) {
  uint32_t activeCount = 0;
  Ptr<VehicleMobilityModel> m = lane.path->GetFront();
  uint32_t indexAtLane = 0;
  while (m!=0) {
    Ptr<Node> n = m->GetNode();
    uint32_t nid = n->GetId();
    //NS_LOG_DEBUG (Simulator::Now().GetSeconds() << " [RoadTrafficScenario::Lane::Update] nid="<< nid);
    if (m->IsActive()) {
      m->Update();
      if (!m_data[nid].isManual && m_driverInput!=0 && m_driverModel!=0) {
        DriverInput::ProjectedVehicleState pvs;
        pvs.laneId = lane.path->GetId();
        RoadTrafficScenario::VehicleState& vs = m_data[nid].projectedState;
        if ( m_data[nid].projectedTime < Simulator::Now() ) {
          m_data[nid].projectedTime = Simulator::Now();
          vs.acceleration = m->GetAcceleration();
          vs.position = m->GetOffsetAlongPath();
          vs.speed = m->GetSpeed();
        }
        pvs.dt = m_data[nid].projectedTime - Simulator::Now();
        pvs.acceleration = m_data[nid].projectedState.acceleration;
        pvs.offset = m_data[nid].projectedState.position;
        pvs.speed = m_data[nid].projectedState.speed;
        m_driverInput->SetHost(m, pvs);
        DriverModel::Action action;
        //if (hasAction && (action.acceleration != m->GetAcceleration())) {
        if (m_driverModel->CalculateAction(*m_driverInput, action)) {
          //NS_LOG_DEBUG ("@" << Simulator::Now().GetSeconds() << " [RoadTrafficScenario::Lane::Update] ScheduleChangeAcceleration nodeId=" << nid << " action.acceleration="<<action.acceleration<<" action.delay="<<action.delay);
          Time t = CalculateReactionTime(nid) + Seconds(action.delay);
          ScheduleChangeAcceleration ( n, action.acceleration, t );
        }
      }
      //NS_LOG_DEBUG ("n="<<n->GetId()<< ", Pos=" << m->GetPosition());
      // check collision with leading vehicle
      Ptr<VehicleMobilityModel> ml = m->GetLeader();
      if (ml != 0) {
        if (ml->GetOffsetAlongPath() <= lane.path->GetLength()) {
          if ( (m->GetOffsetAlongPath() + 0.5 * m->GetLength() + 0.5 * ml->GetLength()) >= ml->GetOffsetAlongPath() ) {
            if (!m->isCollided) { //collision!!
              NS_LOG_DEBUG ("@" << Simulator::Now().GetSeconds() << "s [RoadTrafficScenario::Lane::Update] Collision: nodeId=" << nid << " laneId=" << lane.path->GetId() << " indexAtLane=" << indexAtLane <<
                            " a=" << m->GetAcceleration() << " v=" << m->GetSpeed() << " p=" << m->GetPosition() << " o=" << m->GetOffsetAlongPath()
              );
              m->isCollided = true;
              if (!m_notifyCollision.IsNull()) m_notifyCollision(n, m->GetSpeed(), m->GetSpeed()-ml->GetSpeed());
            }
            // adjust crashed vechicle state
            m->SetOffsetAlongPath( ml->GetOffsetAlongPath() - ( 0.5 * m->GetLength() + 0.5 * ml->GetLength() ) );
            m->SetSpeed( ml->GetSpeed() );
            m->SetAcceleration( ml->GetAcceleration() );
            m_data[nid].isManual = true;
          }
        }
      }
    //check if the node has left the simulation area limit or not moving)
      if (m->GetSpeed() > 0.0f) {
        activeCount++;
      }
    }
    m = m->GetFollower();
    ++indexAtLane;
  }
  return activeCount;
}


uint32_t 
RoadTrafficScenario::FindNodeId(uint32_t laneId, uint32_t indexAtLane) const {
  const Lane& lane = m_lanes.at(laneId);
  if (lane.path->GetFront()!=0 || lane.path->GetBack()!=0) {
    Ptr<VehicleMobilityModel> m = lane.path->GetFront();
    uint32_t i = 0;
    while (m!=0) {
      if (i == indexAtLane) {
        Ptr<Node> n = m->GetNode();
        return n->GetId();
      }
      m = m->GetFollower();
      i++;
    }
  }
  throw std::out_of_range("Node not found.");
}

uint32_t 
RoadTrafficScenario::SetupLane(RoadTrafficScenario::Lane& lane, uint32_t startingNodeId, NodeContainer& nodes) {
  uint32_t nodeId = startingNodeId;
  //initialize storage for new instance of scenario
  uint32_t vcount = lane.initialStates.size();
  NS_ASSERT (lane.path->GetBack()==0);
  NS_ASSERT (lane.path->GetFront()==0);
  //Ptr<VehicleMobilityModel> model1=0;
  for (uint32_t indexAtLane=0; indexAtLane<vcount; ++indexAtLane) {
    VehicleState& vs = lane.initialStates[indexAtLane];
    NS_ASSERT (nodeId < nodes.GetN());
    Ptr<Node> node = nodes.Get(nodeId);
    //Ptr<VehicleMobilityModel> model = mobilityFactory.Create()->GetObject<VehicleMobilityModel>();
    Ptr<VehicleMobilityModel> model = CreateObject<VehicleMobilityModel>();
    if (model == 0) { 
      NS_FATAL_ERROR ("[RoadTrafficScenario::Lane::Setup] FATAL ERROR: VehicleMobilityModel cannot be created."); 
      break;
    }
    //model->isActive = true;
    model->isCollided = false;
    //model->isRemoved = false;
    //model->SetLaneId(lane.laneId);
    //NS_LOG_DEBUG ("path= "<< path->ToString());
    model->SetLanePath(lane.path);
    //model->SetIndexAtLane(indexAtLane);
    //model->SetPosition(vs.position);
    model->SetOffsetAlongPath(vs.position);
    model->SetSpeed(vs.speed);
    model->SetAcceleration(vs.acceleration);
    NS_ASSERT( vs.vehicleType!=0 );
    model->SetVehicleType(vs.vehicleType);
    model->SetActiveStatusChangeCallback(m_notifyActiveStatusChange);
    //NS_LOG_DEBUG ("[RoadTrafficScenario::Lane::Setup] model->GetPosition()= "<< model->GetPosition());
    
    node->AggregateObject(model);
    m_data[nodeId].projectedState.vehicleType = vs.vehicleType;
    m_data[nodeId].projectedState.acceleration = vs.acceleration;
    m_data[nodeId].projectedState.speed = vs.speed;
    m_data[nodeId].projectedState.position = vs.position;
    m_data[nodeId].projectedTime = Seconds(0.0f);
    //vehicles[indexAtLane] = node;
    //collisionDistance[indexAtLane] = std::numeric_limits<double>::infinity();
    nodeId++;
    lane.path->AddBack(model);
/*    model->SetLeader(model1);
    if (model1!=0) model1->SetFollower(model);
    model1 = model;*/
    
  }
  return nodeId;
}



