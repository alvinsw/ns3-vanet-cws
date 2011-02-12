/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver-input.h"
#include "lane-path.h"
#include "ns3/assert.h"
#include "ns3/log.h"

NS_OBJECT_ENSURE_REGISTERED (DriverInput);
NS_LOG_COMPONENT_DEFINE ("DriverInput");

TypeId 
DriverInput::GetTypeId(void )
{
  static TypeId tid = TypeId("DriverInput")
    .SetParent<Object> ()
    //.AddConstructor<DriverInput> ()
    ;
  return tid;
}

DriverInput::DriverInput() : m_pvs(), m_vmm(), m_rts()
{ }

DriverInput::~DriverInput() { }

Ptr< RoadTrafficScenario > DriverInput::GetRTS() const { return m_rts; }
void DriverInput::SetRTS(Ptr< RoadTrafficScenario > rts) { m_rts = rts; }

void DriverInput::SetHost(Ptr< VehicleMobilityModel > mm, const DriverInput::ProjectedVehicleState& state)
{
  m_vmm = mm;
  m_pvs = state;
  m_node = mm->GetNode();
}

void DriverInput::GetProjectedVehicleState(DriverInput::ProjectedVehicleState& state) const
{
  state = m_pvs;
}

double DriverInput::GetProjectedVehicleAcceleration() const
{
  return m_pvs.acceleration;
}

double DriverInput::GetProjectedVehicleOffset() const
{
  return m_pvs.offset;
}
double DriverInput::GetProjectedVehicleSpeed() const
{
  return m_pvs.speed;
}

Time DriverInput::GetProjectedVehicleTime() const
{
  return m_pvs.dt;
}

Ptr< VehicleMobilityModel > DriverInput::GetVehicleMobilityModel() const
{
  return m_vmm;
}
double DriverInput::GetVehicleAcceleration() const
{
  return m_vmm->GetAcceleration();
}
double DriverInput::GetVehicleOffset() const
{
  return m_vmm->GetOffsetAlongPath();
}
double DriverInput::GetVehicleSpeed() const
{
  return m_vmm->GetSpeed();
}

double 
DriverInput::GetVehicleLength() const {
  return m_vmm->GetLength();
}
uint32_t DriverInput::GetVehicleLaneId() const
{
  return m_vmm->GetLanePath()->GetId();
}
Time DriverInput::GetReactionTime() const
{
  return m_rts->CalculateReactionTime(m_node->GetId());
}

bool 
DriverInput::AllowsLeftLaneChange() const {
  if (m_rts->GetNextLaneId(GetVehicleLaneId(), RoadTrafficScenario::LEFT) >= 0) return true;
  else return false;
}

bool 
DriverInput::AllowsRightLaneChange() const {
  if (m_rts->GetNextLaneId(GetVehicleLaneId(), RoadTrafficScenario::RIGHT) >= 0) return true;
  else return false;
}

void DriverInput::DoDispose(void )
{
  m_node = 0;
  m_rts = 0;
  m_vmm = 0;
  Object::DoDispose();
}


NS_OBJECT_ENSURE_REGISTERED (ActualDriverInput);

TypeId 
ActualDriverInput::GetTypeId(void )
{
  static TypeId tid = TypeId("ActualDriverInput")
    .SetParent<DriverInput> ()
    .AddConstructor<ActualDriverInput> ()
    ;
  return tid;
}

ActualDriverInput::ActualDriverInput(uint32_t maxLeadingVehicles, uint32_t maxFollowingVehicles) :
    DriverInput(), m_maxLeadingVehicles(maxLeadingVehicles), m_maxFollowingVehicles(maxFollowingVehicles)
{ }

ActualDriverInput::~ActualDriverInput() { }


uint32_t 
ActualDriverInput::GetMaxFollowingVehicles() const { return m_maxFollowingVehicles; }
uint32_t 
ActualDriverInput::GetMaxLeadingVehicles() const { return m_maxLeadingVehicles; }
void 
ActualDriverInput::SetMaxVehicles(uint32_t leading, uint32_t following) {
  m_maxLeadingVehicles = leading;
  m_maxFollowingVehicles = following;
}

bool 
ActualDriverInput::GetFollowingVehicle(DriverInput::VehicleState& state, uint32_t index) const {
  if (index < m_maxFollowingVehicles) {
    uint32_t i = 0;
    Ptr<VehicleMobilityModel> m = GetVehicleMobilityModel();
    Ptr<VehicleMobilityModel> mf = m->GetFollower();
    while (mf != 0) {
      if (i==index) {
        Ptr<Node> n = mf->GetNode();
        state.nodeId = n->GetId();
        state.acceleration = mf->GetAcceleration();
        state.speed = mf->GetSpeed();
        state.gap = m->GetOffsetAlongPath() - mf->GetOffsetAlongPath() - m->GetLength()/2 - mf->GetLength()/2;
        return true;
      }
      i++;
      mf = mf->GetFollower();    
    }
  }
  return false;
}

bool 
ActualDriverInput::GetLeadingVehicle(DriverInput::VehicleState& state, uint32_t index) const {
  if (index < m_maxLeadingVehicles) {
    uint32_t i = 0;
    Ptr<VehicleMobilityModel> m = GetVehicleMobilityModel();
    Ptr<VehicleMobilityModel> ml = m->GetLeader();
    //no other leader in this segment, stop if approaching end of path
    if (!ml && index == 0) {
      state.nodeId = -1;
      state.acceleration = 0.0f;
      state.speed = 0.0f;
      state.gap = m->GetLanePath()->GetLength() - m->GetOffsetAlongPath() - m->GetLength()/2;
      return true;
    }
    while (ml != 0) {
      if (i==index) {
        Ptr<Node> n = ml->GetNode();
        state.nodeId = n->GetId();
        state.acceleration = ml->GetAcceleration();
        state.speed = ml->GetSpeed();
        state.gap = ml->GetOffsetAlongPath() - m->GetOffsetAlongPath() - m->GetLength()/2 - ml->GetLength()/2;
        return true;
      }
      i++;
      ml = ml->GetLeader();    
    }
  }
  return false;
}

std::pair< bool, bool > ActualDriverInput::GetLeftLaneVehicles(DriverInput::VehicleState& back, DriverInput::VehicleState& front) const {
  Ptr<const LanePath> lanePath = GetRTS()->GetLane( GetRTS()->GetNextLaneId(GetVehicleLaneId(), RoadTrafficScenario::LEFT) ).path;
  return GetLaneVehicles(*lanePath, back, front);
}

std::pair< bool, bool > ActualDriverInput::GetRightLaneVehicles(DriverInput::VehicleState& back, DriverInput::VehicleState& front) const {
  Ptr<const LanePath> lanePath = GetRTS()->GetLane( GetRTS()->GetNextLaneId(GetVehicleLaneId(), RoadTrafficScenario::RIGHT) ).path;
  return GetLaneVehicles(*lanePath, back, front);
}

std::pair<bool,bool> 
ActualDriverInput::GetLaneVehicles(const LanePath& lane, DriverInput::VehicleState& back, DriverInput::VehicleState& front) const {
  bool frontExist = false;
  bool backExist = false;
  Ptr<VehicleMobilityModel> m = GetVehicleMobilityModel();
  Ptr<VehicleMobilityModel> mback = lane.GetFront();
  Ptr<VehicleMobilityModel> mfront;
  while (mback != 0) {
    if (m->GetOffsetAlongPath() == mback->GetOffsetAlongPath()) {
      mfront = mback;
      break;
    } else if (m->GetOffsetAlongPath() > mback->GetOffsetAlongPath()) {
      mfront = mback->GetLeader();
      break;
    }
    mfront = mback;
    mback = mback->GetFollower();
  }
  if (mback != 0) {
    backExist = true;
    Ptr<Node> n = mback->GetNode();
    back.nodeId = n->GetId();
    back.acceleration = mback->GetAcceleration();
    back.speed = mback->GetSpeed();
    back.gap = m->GetOffsetAlongPath() - mback->GetOffsetAlongPath() - m->GetLength()/2 - mback->GetLength()/2;
  }      
  if (mfront != 0) {
    frontExist = true;
    Ptr<Node> nl = mfront->GetNode();
    front.nodeId = nl->GetId();
    front.acceleration = mfront->GetAcceleration();
    front.speed = mfront->GetSpeed();
    front.gap = mfront->GetOffsetAlongPath() - m->GetOffsetAlongPath() - m->GetLength()/2 - mfront->GetLength()/2;
  }
  return std::pair<bool,bool>(backExist, frontExist);
}


