#include "ns3/simulator.h"
#include "ns3/log.h"
#include "vehicle-mobility-model.h"
#include "lane-path.h"
#include "vector-utils.hpp"
#include "cpp/Math.hpp"

NS_LOG_COMPONENT_DEFINE ("VehicleMobilityModel");
NS_OBJECT_ENSURE_REGISTERED (VehicleMobilityModel);

TypeId 
VehicleMobilityModel::GetTypeId (void) {
  static TypeId tid = TypeId ("VehicleMobilityModel")
    .SetParent<ns3::ConstantPositionMobilityModel> ()
    .AddConstructor<VehicleMobilityModel> ()
    ;
  return tid;
}

VehicleMobilityModel::VehicleMobilityModel() : 
    isCollided(false), m_node(), m_follower(), m_leader(),  //m_indexAtLane(0), m_laneId(0), 
    m_lastUpdate(), m_position(), m_speed(0), m_acceleration(0),
    m_offsetAlongPath(0), m_type(0), m_lanePath(0),
    m_isActive(true), m_notifyActiveStatusChange()
{}

VehicleMobilityModel::~VehicleMobilityModel()
{}

Ptr< Node > VehicleMobilityModel::GetNode() const
{
  return m_node;
}

double 
VehicleMobilityModel::GetSpeed() const {
  Update();
  return m_speed;
}

void 
VehicleMobilityModel::SetSpeed(double speed) {
  Update();
  if (m_speed != speed) {
    m_speed = speed;
    NotifyCourseChange ();
  }
}

void 
VehicleMobilityModel::SetAcceleration(double acceleration) {
  Update();
  if (m_acceleration != acceleration) {
    m_acceleration = acceleration;
    NotifyCourseChange ();
  }
  NS_LOG_DEBUG ("@" << Simulator::Now() << "[VehicleMobilityModel::SetAcceleration] o="<<m_offsetAlongPath<< ", p=" << m_position <<", v=" << m_speed << ", a=" << m_acceleration);
}

double VehicleMobilityModel::GetAcceleration() const {
  Update();
  return m_acceleration;
}

double VehicleMobilityModel::GetLength() const { return m_type->GetLength(); }
double VehicleMobilityModel::GetWidth() const { return m_type->GetWidth(); }

uint32_t VehicleMobilityModel::GetLaneId() const { return m_lanePath->GetId(); }
//void VehicleMobilityModel::SetLaneId(uint32_t id) { m_laneId = id; }

//uint32_t VehicleMobilityModel::GetIndexAtLane() const { return m_indexAtLane; }
//void VehicleMobilityModel::SetIndexAtLane(uint32_t index) { m_indexAtLane = index; }

Ptr< VehicleMobilityModel > VehicleMobilityModel::GetFollower() const { return m_follower; }
void VehicleMobilityModel::SetFollower(Ptr< VehicleMobilityModel > m) { m_follower = m; }

Ptr< VehicleMobilityModel > VehicleMobilityModel::GetLeader() const { return m_leader; }
void VehicleMobilityModel::SetLeader(Ptr< VehicleMobilityModel > m) { m_leader = m; }

Ptr< VehicleType > VehicleMobilityModel::GetVehicleType() const { return m_type; }
void VehicleMobilityModel::SetVehicleType(Ptr<VehicleType> vtype) { m_type = vtype; }

Ptr<LanePath> VehicleMobilityModel::GetLanePath() const { return m_lanePath; }

void VehicleMobilityModel::SetLanePath(Ptr<LanePath> lanePath, double offset) { 
  if (m_lanePath != lanePath) {
    m_lanePath = lanePath;
    m_offsetAlongPath = offset;
    m_position = m_lanePath->CalculatePoint(m_offsetAlongPath);
  }
}

double VehicleMobilityModel::GetOffsetAlongPath() const {
  Update();
  return m_offsetAlongPath;
}

void VehicleMobilityModel::SetOffsetAlongPath(double offset) {
  Update();
  if (m_offsetAlongPath != offset) {
    NS_ASSERT(m_lanePath!=0);
    m_offsetAlongPath = offset;
    if (m_offsetAlongPath >= m_lanePath->GetLength()) {
      //Ptr<Node> n = this->GetObject<Node>();
      //NS_LOG_WARN("Node=" << n->GetId() << ": starting offset exceeded lane path length.");
      NS_LOG_WARN("[VehicleMobilityModel] WARN: Offset exceeded lane path length.");
    }
    m_position = m_lanePath->CalculatePoint(m_offsetAlongPath);
    //NS_LOG_DEBUG ("[VehicleMobilityModel::SetOffsetAlongPath] m_position= "<< m_position);
    NotifyCourseChange();
  }
}

Vector VehicleMobilityModel::GetDirection() const {
  Update();
  return m_lanePath->GetDirection(m_offsetAlongPath);
}

double VehicleMobilityModel::GetHeading() const {
  const double& pi = cpp::math::PI;
  Vector dir = GetDirection();
  if (dir.x>0) {
    if (dir.y>0) { //0<a<90
      return atan(dir.x/dir.y);
    } else if (dir.y<0) { //90<a<180
      return pi + atan(dir.x/dir.y);
    } else { //y==0, a=90
      return pi/2;
    }
  } else if (dir.x<0) {
    if (dir.y>0) { //270<a<360
      return 2*pi + atan(dir.x/dir.y);
    } else if (dir.y<0) { //180<a<270
      return pi + atan(dir.x/dir.y);
    } else { //y==0, a=270
      return pi + pi/2;
    }
  } else { //x==0
    if (dir.y>0) { //a=0
      return 0;
    } else if (dir.y<0) { //a=180
      return pi;
    } else { //y==0, error
      return 0;
    }
  }
}

Vector
VehicleMobilityModel::DoGetPosition() const {
  Update();
  return m_position;
}

void 
VehicleMobilityModel::DoSetPosition(const Vector& position) {
  Update();
  if (Equals(m_position, position)) {
    m_position = position;
  } else {
    m_position = position;
    NotifyCourseChange ();
  }
}

Vector
VehicleMobilityModel::DoGetVelocity() const {
  Update();
  return m_lanePath->GetDirection(m_offsetAlongPath) * m_speed;
}

void VehicleMobilityModel::Update() const {
  if ( m_lastUpdate < Simulator::Now() ) {
    if (m_isActive && (m_speed > 0.0f || m_acceleration > 0.0f)) {
      double t = (Simulator::Now () - m_lastUpdate).GetSeconds();
      double half_t_square = t*t*0.5;
      m_offsetAlongPath += (m_speed*t + m_acceleration*half_t_square);
      m_position = m_lanePath->CalculatePoint(m_offsetAlongPath);
      m_speed += (m_acceleration*t);
      // check if the velocity is 0 or less than 0, make it 0
      if ( m_speed <= 0.0f ) {
        m_speed = 0.0f;
        m_acceleration = 0.0f;
        //isActive = false;
      }
      m_lastUpdate = Simulator::Now();
      //NS_LOG_DEBUG ("[VehicleMobilityModel::Update()] m_offsetAlongPath="<<m_offsetAlongPath<< ", m_position=" << m_position <<", m_speed=" << m_speed << ", m_acceleration=" << m_acceleration);
      // check if the node left the simulation area limit
      if (m_offsetAlongPath > m_lanePath->GetLength()) {
        //lanepath = route->GetNext(m_lanePath)
        //if (lanepath!=0) else
        Deactivate();
      }
    }
  }
}

double VehicleMobilityModel::GetOffsetFrom(Ptr<VehicleMobilityModel> m) const {
  NS_ASSERT( m != 0 );
  NS_ASSERT( GetLaneId() == m->GetLaneId() );
  return this->GetOffsetAlongPath() - m->GetOffsetAlongPath();
}

void VehicleMobilityModel::Activate() const
{
  if (!m_isActive) {
    m_position.z = 0.0f;
    m_isActive = true;
    NotifyActiveStatusChange(true);
  }
}
void VehicleMobilityModel::Deactivate() const
{
  if (m_isActive) {
    m_position.z = 10000.0f;
    m_isActive = false;
    NotifyActiveStatusChange(false);
  }
}

bool VehicleMobilityModel::IsActive() const
{
  return m_isActive;
}

void VehicleMobilityModel::NotifyActiveStatusChange(bool isActive) const
{
  Ptr<VehicleMobilityModel> m = const_cast<VehicleMobilityModel*>(this);
  if (!m_notifyActiveStatusChange.IsNull()) m_notifyActiveStatusChange(m, isActive);
}

void VehicleMobilityModel::SetActiveStatusChangeCallback(VehicleMobilityModel::ActiveStatusChangeCallback callback)
{
  m_notifyActiveStatusChange = callback;
}

void VehicleMobilityModel::NotifyNewAggregate(void )
{
  m_node = GetObject<Node>();
  Object::NotifyNewAggregate();
}

void VehicleMobilityModel::DoDispose(void )
{
  m_node = 0;
  m_follower = 0;
  m_leader = 0;
  m_lanePath = 0;
  m_type = 0;
  Object::DoDispose();
}
