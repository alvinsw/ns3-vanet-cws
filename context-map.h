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

#ifndef CONTEXTMAP_H
#define CONTEXTMAP_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/node-container.h"
#include "vector-utils.hpp"
#include "sae-types.hpp"
#include "driver-input.h"
#include "vehicle-mobility-model.h"
#include "cpp/graph/DirectedGraphAdjacencyList.hpp"
#include <vector>
#include <map>

using namespace ns3;

struct StateItem {
  StateItem() : state(), timestamp(), offset(), laneId() {}
  StateItem(sae::VehicleState state_, ns3::Time timestamp_, double offset_, double laneId_) 
    : state(state_), timestamp(timestamp_), offset(offset_), laneId(laneId_) {}
  sae::VehicleState state;
  ns3::Time timestamp;
  double offset; //offset of a vehicle in a path in meters
  uint32_t laneId;
  uint32_t sumDL; //sum of danger level in micro units (10^6)
  ns3::Time rsmDelay;
};

// struct VehicleInteraction {
//   Time urgencyTime;
// };

/** Context map for each node */
class ContextMap : public Object {
  public:
    //typedef std::map< uint32_t, StateItem > TStateMap;
    //typedef TStateMap::iterator TStateMapIterator;
    //typedef TStateMap::const_iterator TStateMapConstIterator;
    
    /** Positive value in m/s2 */
    static double defaultVehicleDecelMax;
    /** Positive value in m/s2 */
    static double defaultVehicleDecelNormal;
    /** In seconds */
    static Time defaultReactionTimeMax;
    /** In seconds */
    static Time defaultReactionTimeMin;
    /** In meters */
    static double defaultMinGap;
    static Time maxStateLifetime;
    
    static TypeId GetTypeId(void);
    static void Install(const NodeContainer& nodes);

    ContextMap();
    ContextMap(uint32_t maxNodeId);
    virtual ~ContextMap();
    
    /** MaxNodeId is the highest node Id available in the simulation. */
    void SetMaxNodeId(uint32_t maxId);
    void GetHostState(StateItem& state);
    bool GetState(uint32_t vehicleId, StateItem& state) const;
    Time GetRsmDelay(uint32_t vehicleId) const;
    void SetRsmDelay(uint32_t vehicleId, Time delay);
    void UpdateState(uint32_t vehicleId, const StateItem& state);
    void UpdateHostState(Time currentTime);
    void UpdateInteractionGraph(uint32_t id1, const StateItem& state1, uint32_t id2, const StateItem& state2);
    uint32_t GetMaxSumDL() const;
    uint32_t Count() const;
    //TStateMapIterator GetStateBegin();
    //TStateMapIterator GetStateEnd();
    uint32_t GetDL(uint32_t vehicleId1, uint32_t vehicleId2);
    void CalculateSumDL(uint32_t& sumDL, uint32_t& selfDL, uint32_t hostId) const;
    void CalculateDistanceError(Callback< void, double > cb) const;
    
protected:
    virtual void NotifyNewAggregate(void );
  
  private:
    
    class ContextItem : public SimpleRefCount<ContextItem> {
    public:
      ContextItem();
      StateItem stateItem;
      cpp::graph::IVertex<uint32_t,uint32_t>* vertex;
    };
    
    class CVertexMap : public cpp::Object {
    public:
      typedef cpp::graph::IVertex<uint32_t,uint32_t>* VertexPtr;
      typedef uint32_t Locator;
      CVertexMap(uint32_t size=0);
      virtual ~CVertexMap();
      VertexPtr Get(uint32_t key) const;
      uint32_t GetKey(Locator loc) const;
      const StateItem& GetStateItem(uint32_t key) const;
      StateItem& GetStateItem(uint32_t key);
      void Set(uint32_t key, const VertexPtr& value);
      void Set(uint32_t key, const StateItem& value);
      bool Resize(uint32_t newSize);
      Locator Add(uint32_t key, const VertexPtr& value);
      void Remove(Locator loc);
      void Clear();
      std::vector< Ptr<ContextItem> > m_items;
    };
    typedef cpp::graph::DirectedGraphAdjacencyList<uint32_t, uint32_t, CVertexMap> TInteractionGraph;
    
    CVertexMap m_map;
    TInteractionGraph m_ig;
    Ptr<VehicleMobilityModel> m_vmm;
    uint32_t m_hostId;
};



class CwsDriverInput : public DriverInput {
  public:
    CwsDriverInput();
    virtual ~CwsDriverInput();
    
    virtual bool GetLeadingVehicle(VehicleState& state, uint32_t index = 0) const;
    virtual bool GetFollowingVehicle(VehicleState& state, uint32_t index = 0) const;
    virtual std::pair<bool,bool> GetLeftLaneVehicles(VehicleState& back, VehicleState& front) const;
    virtual std::pair<bool,bool> GetRightLaneVehicles(VehicleState& back, VehicleState& front) const;
};

#endif // CONTEXTMAP_H
