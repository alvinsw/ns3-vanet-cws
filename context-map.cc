/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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

#include "context-map.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
//#include "cpp/Float.hpp"
//#include "lane-path.h"
#include "default-parameters.h"

NS_LOG_COMPONENT_DEFINE ("ContextMap");
NS_OBJECT_ENSURE_REGISTERED (ContextMap);

double ContextMap::defaultVehicleDecelMax = DP_VEH_MAX_DECEL;
double ContextMap::defaultVehicleDecelNormal = DP_VEH_DECEL;
double ContextMap::defaultMinGap = DP_VEH_MIN_GAP;
Time ContextMap::defaultReactionTimeMax = Seconds(2.5f);
Time ContextMap::defaultReactionTimeMin = Seconds(0.2f);
Time ContextMap::maxStateLifetime = Seconds(10.0f);

TypeId 
ContextMap::GetTypeId(void) {
  static TypeId tid = TypeId("ContextMap")
    .SetParent<Object> ()
    .AddConstructor<ContextMap> ()
    ;
  return tid;
}

void ContextMap::Install(const ns3::NodeContainer& nodes) {
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<Node> n = nodes.Get(i);
    Ptr<ContextMap> cm = CreateObject<ContextMap>(nodes.GetN());
    n->AggregateObject(cm);
  }
}

ContextMap::ContextMap() : m_map(), m_ig(0, cpp::Ref<CVertexMap>(&m_map)) {}
ContextMap::ContextMap ( uint32_t maxNodeId ) : m_map( maxNodeId ), m_ig(0, cpp::Ref<CVertexMap>(&m_map)) { }
ContextMap::~ContextMap() {}

void 
ContextMap::SetMaxNodeId ( uint32_t maxId )
{
  m_map.Resize(maxId);
}

void 
ContextMap::GetHostState(StateItem& state)
{
  if ( !GetState(m_hostId, state) ) {
    UpdateHostState(Simulator::Now());
  }
}

bool 
ContextMap::GetState ( uint32_t vehicleId, StateItem& state ) const
{
  if (m_map.m_items[vehicleId] == 0) return false;
  else {
    state = m_map.m_items[vehicleId]->stateItem;
    return true;
  }
}

Time ContextMap::GetRsmDelay(uint32_t vehicleId) const
{
  return m_map.GetStateItem(vehicleId).rsmDelay;
}
void ContextMap::SetRsmDelay(uint32_t vehicleId, Time delay)
{
  m_map.GetStateItem(vehicleId).rsmDelay = delay;
}

uint32_t ContextMap::GetMaxSumDL() const {
  uint32_t maxSumDL = 0;
  TInteractionGraph::VertexIterator vi = m_ig.GetVertices();
  while (vi.HasNext()) {
    TInteractionGraph::Vertex vx = vi.Next();
    const StateItem& si = m_map.GetStateItem( vx->GetValue() );
    if (si.sumDL > maxSumDL) maxSumDL = si.sumDL;
  }
  return maxSumDL;
}

uint32_t ContextMap::Count() const {
  return m_ig.VerticesSize();
}

// ContextMap::TStateMapIterator ContextMap::GetStateBegin() {
//   return stateMap.begin();
// }
// ContextMap::TStateMapIterator ContextMap::GetStateEnd() {
//   return stateMap.end();
// }

void ContextMap::UpdateState(uint32_t vehicleId, const StateItem& state) {
  TInteractionGraph::Vertex v1 = m_ig.AddVertex(vehicleId);
  m_map.Set(vehicleId, state);
  TInteractionGraph::VertexIterator vi = m_ig.GetVertices();
  while (vi.HasNext()) {
    TInteractionGraph::Vertex v2 = vi.Next();
    if (v1!=v2) {
      uint32_t id2 = v2->GetValue();
      const StateItem& si2 = m_map.GetStateItem(vehicleId);
      UpdateInteractionGraph (vehicleId, state, id2, si2);
      if ((state.timestamp - si2.timestamp) > maxStateLifetime) {
        m_ig.RemoveVertex(*v2);
      }      
    }
  }
  //std::cout << ig << std::endl;
}

void ContextMap::UpdateHostState(Time currentTime)
{
  const Vector& pos = m_vmm->GetPosition();
  sae::VehicleState state(pos.x, pos.y, m_vmm->GetSpeed(), m_vmm->GetHeading(), m_vmm->GetAcceleration(), m_vmm->GetLength(), m_vmm->GetWidth());
  uint32_t sumDL = 0;
  uint32_t selfDL = 0;
  CalculateSumDL(sumDL, selfDL, m_hostId);
  StateItem si(state, currentTime, m_vmm->GetOffsetAlongPath(), m_vmm->GetLaneId() );
  UpdateState(m_hostId, si);
}

void ContextMap::UpdateInteractionGraph (uint32_t id1, const StateItem& state1, uint32_t id2, const StateItem& state2)
{
  const sae::VehicleState& vstate1 = state1.state;
  const sae::VehicleState& vstate2 = state2.state;
  if (state1.laneId == state2.laneId) {
    double vL, vF;
    uint32_t leader_id, follower_id;
    if (state1.offset > state2.offset) { //larger offset means at front
      //leader
      vL = vstate1.speed.GetMetersPerSecond(); //leader
      //follower
      vF = vstate2.speed.GetMetersPerSecond(); //follower
      leader_id = id1;
      follower_id = id2;
    } else {
      //leader
      vL = vstate2.speed.GetMetersPerSecond(); //leader
      //follower
      vF = vstate1.speed.GetMetersPerSecond(); //follower
      leader_id = id2;
      follower_id = id1;
    }
    TInteractionGraph::Vertex sv = m_ig.GetVertex(leader_id);
    TInteractionGraph::Vertex tv = m_ig.GetVertex(follower_id);
    m_ig.RemoveEdges(*sv, *tv);
    if (vF > 0.0001f) {
      double decL = defaultVehicleDecelMax; //leader
      double decF = defaultVehicleDecelNormal;  //follower
      //double safeDistance = defaultMinGap + vF * defaultReactionTimeMax + 0.5 * ( vF*vF/aF - vL*vL/aL ) + state1.size.length/2 + state2.size.length/2;
      double x1 = vstate1.position.GetXinMeters();
      double y1 = vstate1.position.GetYinMeters();
      double x2 = vstate2.position.GetXinMeters();
      double y2 = vstate2.position.GetYinMeters();
      double actualDistance = cpp::math::geom::Distance<double>(x1,y1,x2,y2);
      actualDistance = actualDistance - defaultMinGap - (vstate1.size.GetLengthInMeters()/2 + vstate2.size.GetLengthInMeters()/2);
      // available reaction time to avoid a collision in case of sudden braking of leader vehicle
      //double reactionTimeAvail = 0; 
      double reactionTimeAvail = (actualDistance - (0.5 * (vF*vF/decF - vL*vL/decL))) / vF; 
      //std::cout << "reactionTimeAvail:"<<reactionTimeAvail << std::endl;
      if (vL < vF) {
        double vRel = vF - vL;
        // available reaction time to avoid a collision in case of follower moving faster than leader
        double reactionTimeAvail2 = (actualDistance - (0.5 * vRel*vRel/decF) ) / vRel;
        //std::cout << "reactionTimeAvail2:"<<reactionTimeAvail << std::endl;
        if (reactionTimeAvail > reactionTimeAvail2) reactionTimeAvail = reactionTimeAvail2;
      }
      //std::cout << "reactionTimeAvail="<<reactionTimeAvail << std::endl;
      Time rt = Seconds(reactionTimeAvail);
      if (rt < defaultReactionTimeMax) {
        if (rt < defaultReactionTimeMin) rt = defaultReactionTimeMin;
        double DL1 = 1 - ((rt.GetSeconds() - defaultReactionTimeMin.GetSeconds()) / (defaultReactionTimeMax.GetSeconds() - defaultReactionTimeMin.GetSeconds()));
        uint32_t DL = (uint32_t)(DL1 * 1000000.0f);
        if (DL <= 0) DL = 1; //smallest DL
        m_ig.AddEdge(*sv, *tv, DL);
      }
      //std::cout << "edge:" <<  ig.GetEdgeValue(leader_id, follower_id).GetSeconds() << std::endl;;
    }
  }

}

uint32_t ContextMap::GetDL(uint32_t vehicleId1, uint32_t vehicleId2) {
  return m_ig.GetEdgeValue(vehicleId1, vehicleId2);
}

void ContextMap::CalculateSumDL(uint32_t& sumDL, uint32_t& selfDL, uint32_t hostId) const {
  selfDL = 0;
  sumDL = 0;
  TInteractionGraph::VertexIterator vi = m_ig.GetVertices();
  while (vi.HasNext()) {
    uint32_t maxDL = 0;
    TInteractionGraph::Vertex vx = vi.Next();
    TInteractionGraph::EdgeIterator ei = m_ig.GetOutEdges(*vx);
    while (ei.HasNext()) {
      TInteractionGraph::Edge e = ei.Next();
      if (e->GetValue() > maxDL) {
        maxDL = e->GetValue();
      }
    }
    sumDL += maxDL;
    if (vx->GetValue() == hostId) selfDL = maxDL;
  }  
}
void ContextMap::DoDispose(void )
{
  //m_map.Clear();
  //m_ig.Clear();
  m_vmm = 0;
  Object::DoDispose ();
}

void ContextMap::NotifyNewAggregate(void )
{
  m_vmm = this->GetObject<VehicleMobilityModel>();
  NS_ASSERT (m_vmm != 0);
  Ptr<Node> n = GetObject<Node>();
  m_hostId = n->GetId();
  Object::NotifyNewAggregate();
}

void ContextMap::CalculateDistanceError(ns3::Callback< void, double > cb) const
{
  TInteractionGraph::VertexIterator vi = m_ig.GetVertices();
  while (vi.HasNext()) {
    TInteractionGraph::Vertex vx = vi.Next();
    uint32_t id = vx->GetValue();
    Ptr<Node> nn = NodeList::GetNode(id);
    Ptr<VehicleMobilityModel> mm = nn->GetObject<VehicleMobilityModel>();
    double actual_x = mm->GetPosition().x;
    double actual_y = mm->GetPosition().y;
    const StateItem& si = m_map.GetStateItem(id);
    double known_x = si.state.position.GetXinMeters();
    double known_y = si.state.position.GetYinMeters();
    double distanceError = cpp::math::geom::Distance<double>(actual_x, actual_y, known_x, known_y);
    cb(distanceError);
  }
}




ContextMap::ContextItem::ContextItem() : stateItem(), vertex() {}

ContextMap::CVertexMap::CVertexMap ( uint32_t size ) : m_items(size) {}
ContextMap::CVertexMap::~CVertexMap() {}

ContextMap::CVertexMap::Locator 
ContextMap::CVertexMap::Add ( uint32_t key, const VertexPtr& value )
{
  Set(key, value);
  return key;
}

void ContextMap::CVertexMap::Clear()
{
  for (uint32_t i=0; i<m_items.size(); ++i) {
    m_items[i] = 0;
  }
}
ContextMap::CVertexMap::VertexPtr 
ContextMap::CVertexMap::Get ( uint32_t key ) const
{
  NS_ASSERT (key < m_items.size());
  //NS_ASSERT (m_items[key]!=0);
  if (m_items[key]!=0) return m_items[key]->vertex;
  return 0;
}
uint32_t ContextMap::CVertexMap::GetKey ( Locator loc ) const
{
  NS_ASSERT (loc <= m_items.size());
  return loc;
}
const StateItem& ContextMap::CVertexMap::GetStateItem ( uint32_t key ) const
{
  NS_ASSERT (key <= m_items.size());
  NS_ASSERT (m_items[key]!=0);
  return m_items[key]->stateItem;
}
StateItem& ContextMap::CVertexMap::GetStateItem(uint32_t key)
{
  NS_ASSERT (key <= m_items.size());
  NS_ASSERT (m_items[key]!=0);
  return m_items[key]->stateItem;
}
void ContextMap::CVertexMap::Remove ( Locator loc )
{
  NS_ASSERT (loc <= m_items.size());
  m_items[loc] = 0;
}
bool ContextMap::CVertexMap::Resize ( uint32_t newSize )
{
  m_items.resize(newSize);
  return true;
}
void ContextMap::CVertexMap::Set ( uint32_t key, const VertexPtr& value )
{
  if (key >= m_items.size()) {
    m_items.resize(key+1);
  }
  if (m_items[key] == 0) {
    m_items[key] = Create<ContextMap::ContextItem>();
  }
  m_items[key]->vertex = value;
}
void ContextMap::CVertexMap::Set ( uint32_t key, const StateItem& value )
{
  if (key >= m_items.size()) {
    m_items.resize(key+1);
  }
  if (m_items[key] == 0) {
    m_items[key] = Create<ContextMap::ContextItem>();
  }
  m_items[key]->stateItem = value;
}

