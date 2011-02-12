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

#include "ns3/log.h"
#include "ns3/assert.h"
#include "cws-driver-input.h"
#include "context-map.h"
#include "lane-path.h"

NS_LOG_COMPONENT_DEFINE ("CwsDriverInput");
NS_OBJECT_ENSURE_REGISTERED (CwsDriverInput);

TypeId 
CwsDriverInput::GetTypeId(void )
{
  static TypeId tid = TypeId("CwsDriverInput")
    .SetParent<DriverInput> ()
    .AddConstructor<CwsDriverInput> ()
    ;
  return tid;
}

CwsDriverInput::CwsDriverInput() { }

CwsDriverInput::~CwsDriverInput() {}

bool CwsDriverInput::GetFollowingVehicle(DriverInput::VehicleState& state, uint32_t index) const {
  if (index > 0) return false;
  Ptr<VehicleMobilityModel> m = GetVehicleMobilityModel();
  Ptr<ContextMap> cm = m->GetObject<ContextMap>();
  Ptr<VehicleMobilityModel> mf = m->GetFollower();
  if (mf != 0) {
    Ptr<Node> n = mf->GetNode();
    StateItem si;
    if ( cm->GetState(n->GetId(), si) ) {
      state.nodeId = n->GetId();
      state.acceleration = si.state.acceleration.GetMetersPerSecondSquared();
      state.speed = si.state.speed.GetMetersPerSecond();
      double x1 = m->GetPosition().x;
      double x2 = si.state.position.GetXinMeters();
      double dist = std::abs(x1-x2);
      state.gap = dist - m->GetLength()/2 - mf->GetLength()/2;
      return true;
    }
  }
  return false;
}

bool CwsDriverInput::GetLeadingVehicle(DriverInput::VehicleState& state, uint32_t index) const {
//std::cout << "GetLeadingVehicle "<< index << std::endl;
  if (index > 0) return false;
  Ptr<VehicleMobilityModel> m = GetVehicleMobilityModel();
  Ptr<ContextMap> cm = m->GetObject<ContextMap>();
  Ptr<VehicleMobilityModel> ml = m->GetLeader();
  if (ml != 0) {
    Ptr<Node> n = ml->GetNode();
    StateItem si;
    if ( cm->GetState(n->GetId(), si) ) {
      state.nodeId = n->GetId();
      state.acceleration = si.state.acceleration.GetMetersPerSecondSquared();
      state.speed = si.state.speed.GetMetersPerSecond();
      double x1 = m->GetPosition().x;
      double x2 = si.state.position.GetXinMeters();
      double dist = std::abs(x1-x2);
      state.gap = dist - m->GetLength()/2 - ml->GetLength()/2;
//std::cout << "state.gap="<< state.gap << std::endl;
      return true;
    }
  }
  //no other leader in this segment, stop if approaching end of path
  state.nodeId = -1;
  state.acceleration = 0.0f;
  state.speed = 0.0f;
  state.gap = m->GetLanePath()->GetLength() - m->GetOffsetAlongPath() - m->GetLength()/2;
  return false;
}

std::pair< bool, bool > CwsDriverInput::GetLeftLaneVehicles(DriverInput::VehicleState& back, DriverInput::VehicleState& front) const {
  return std::pair< bool, bool >(false, false);
}
std::pair< bool, bool > CwsDriverInput::GetRightLaneVehicles(DriverInput::VehicleState& back, DriverInput::VehicleState& front) const {
  return std::pair< bool, bool >(false, false);
}

