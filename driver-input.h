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

#ifndef DRIVER_INPUT_H
#define DRIVER_INPUT_H

#include "ns3/node.h"
#include "ns3/ptr.h"
#include "vehicle-mobility-model.h"
#include "road-traffic-scenario.h"


class DriverInput : public SimpleRefCount<DriverInput> {
  public:
    struct VehicleState {
      explicit VehicleState(uint32_t nodeId_=0, double gap_=0, float speed_=0, float acceleration_=0) : 
          nodeId(nodeId_), gap(gap_), speed(speed_), acceleration(acceleration_) 
      {}
      uint32_t nodeId;
      double gap;
      double speed;
      double acceleration;
    };
    struct ProjectedVehicleState {
      explicit ProjectedVehicleState(Time dt_=MilliSeconds(0), double offset_=0.0f, float speed_=0.0f, float acceleration_=0.0f, uint32_t laneId_=0) : 
          dt(dt_), offset(offset_), speed(speed_), acceleration(acceleration_), laneId(laneId_)
      {}
      Time dt; //time difference in the future, this state is projected at current simulation time + dt
      double offset;
      double speed;
      double acceleration;
      uint32_t laneId;
    };
    
    DriverInput();
    virtual ~DriverInput();
    
    Ptr<RoadTrafficScenario> GetRTS() const;
    void SetRTS(Ptr<RoadTrafficScenario> rts);
    
    void SetHost(Ptr<VehicleMobilityModel> mm, const ProjectedVehicleState& state);
    //Ptr<VehicleMobilityModel> GetVehicle() const;
    void GetProjectedVehicleState(ProjectedVehicleState& state) const;
    double GetProjectedVehicleAcceleration() const;
    double GetProjectedVehicleOffset() const;
    double GetProjectedVehicleSpeed() const;
    Time GetProjectedVehicleTime() const;
    Ptr<VehicleMobilityModel> GetVehicleMobilityModel() const;
    double GetVehicleAcceleration() const;
    double GetVehicleOffset() const;
    double GetVehicleSpeed() const;
    double GetVehicleLength() const;
    uint32_t GetVehicleLaneId() const;
    Time GetReactionTime() const;
    //void SetReactionTime(Time t);
    
    //virtual bool MustStop() const = 0;
    virtual bool AllowsRightLaneChange() const;
    virtual bool AllowsLeftLaneChange() const;
    
    virtual bool GetLeadingVehicle(VehicleState& state, uint32_t index = 0) const = 0;
    virtual bool GetFollowingVehicle(VehicleState& state, uint32_t index = 0) const = 0;
    
    virtual std::pair<bool,bool>  GetLeftLaneVehicles(VehicleState& back, VehicleState& front) const = 0;
    virtual std::pair<bool,bool>  GetRightLaneVehicles(VehicleState& back, VehicleState& front) const = 0;
    
private:
  ProjectedVehicleState m_pvs;
  Ptr<VehicleMobilityModel> m_vmm;
  Ptr<RoadTrafficScenario> m_rts;
  Ptr<Node> m_node;
};

/** The driver aware of the actual state of up to N leading vehicles */
class ActualDriverInput : public DriverInput {
  public:
    ActualDriverInput(uint32_t maxLeadingVehicles=1, uint32_t maxFollowingVehicles=1);
    virtual ~ActualDriverInput();
    /** Default is 1 */
    void SetMaxVehicles(uint32_t leading, uint32_t following);
    uint32_t GetMaxLeadingVehicles() const;
    uint32_t GetMaxFollowingVehicles() const;
    
    virtual bool GetLeadingVehicle(VehicleState& state, uint32_t index = 0) const;
    virtual bool GetFollowingVehicle(VehicleState& state, uint32_t index = 0) const;
    
    virtual std::pair<bool,bool>  GetLeftLaneVehicles(VehicleState& back, VehicleState& front) const;
    virtual std::pair<bool,bool>  GetRightLaneVehicles(VehicleState& back, VehicleState& front) const;
    
  private:
    std::pair<bool,bool> GetLaneVehicles(const LanePath& lane, VehicleState& back, VehicleState& front) const;
    
    uint32_t m_maxLeadingVehicles;
    uint32_t m_maxFollowingVehicles;
};

//class CwsDriverInput : public DriverInput {
//};

#endif // DRIVER_INPUT_H

