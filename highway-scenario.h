#ifndef HIGHWAY_SCENARIO_H
#define HIGHWAY_SCENARIO_H

#include "road-traffic-scenario.h"
#include "lane-path.h"

/** Construct the intial state for a lane */
class LaneInitialState {
  public:
    LaneInitialState(uint32_t vehiclesCount);
    virtual ~LaneInitialState();
    uint32_t VehiclesCount() const;
    virtual void SetupInitialState(std::vector<RoadTrafficScenario::VehicleState>& laneState, LanePath& path) const = 0;
    
    uint32_t pVehiclesCount;
};


class HighwayScenario : public RoadTrafficScenario {
  public:
    HighwayScenario(double highwayLength, double laneWidth, Ptr<VehicleType> vehicleType);
    virtual ~HighwayScenario();
    
    void AddHighwayLane(const LaneInitialState& laneInit, Direction dir);
    double GetHighwayLength() const;
    double GetHighwayLaneWidth() const;
    Ptr<VehicleType> GetVehicleType() const;
    virtual uint32_t GetNextLaneId(uint32_t laneId, Direction side) const;    
  protected:
    virtual uint32_t Setup();
    
  private:
    double m_highwayLength;
    double m_HighwayLaneWidth;
    Ptr<VehicleType> m_VehicleType;
    //std::vector<LaneInitialState*> m_laneIS;
    uint32_t m_vehiclesCount;
    std::vector<Direction> m_lanesDir;
    //std::vector<StraightLanePath> m_lanePaths;
};

//EmptyLane

class NClusterLane : public LaneInitialState {
  public:
    NClusterLane(uint32_t clusterCount, uint32_t vehiclePerCluster, 
                 float interVehicleSpacing, float interClusterSpacing, 
                 float vehicleSpeed = 0, float vehicleAcceleration=0);
    virtual ~NClusterLane();
    void SetupInitialState(std::vector<RoadTrafficScenario::VehicleState>& laneState, LanePath& path) const;
    
    uint32_t pClustersCount;
    uint32_t pVehiclesPerClusterCount;
    float pInterVehicleSpacing;
    float pInterClusterSpacing;
    float pVehicleSpeed;
    float pVehicleAcceleration;
};

class SimpleLane : public LaneInitialState {
  public:
    SimpleLane(uint32_t vehiclesCount, float interVehicleSpacing, float vehicleSpeed = 0, float vehicleAcceleration=0);
    virtual ~SimpleLane();
    
    void SetupInitialState(std::vector<RoadTrafficScenario::VehicleState>& laneState, LanePath& path) const;
    
    float pInterVehicleSpacing;
    float pVehicleSpeed;
    float pVehicleAcceleration;
};

class RandomSpacingLane : public LaneInitialState {
  public:
    RandomSpacingLane(uint32_t vehiclesCount, float minSpacing, float maxSpacing, float vehicleSpeed = 0, float vehicleAcceleration=0);
    virtual ~RandomSpacingLane();
    
    void SetupInitialState(std::vector<RoadTrafficScenario::VehicleState>& laneState, LanePath& path) const;
    
    float pMinSpacing;
    float pMaxSpacing;
    float pVehicleSpeed;
    float pVehicleAcceleration;
};

class LinearSpacingAndSpeedLane : public LaneInitialState {
  public:
    LinearSpacingAndSpeedLane(uint32_t vehiclesCount, float minSpacing, float maxSpacing, float minSpeed = 0, float maxSpeed = 0, float vehicleAcceleration=0);
    virtual ~LinearSpacingAndSpeedLane();
    
    void SetupInitialState(std::vector<RoadTrafficScenario::VehicleState>& laneState, LanePath& path) const;
    
    float pMinSpacing;
    float pMaxSpacing;
    float pMinSpeed;
    float pMaxSpeed;
    float pVehicleAcceleration;
};

#endif /* HIGHWAY_SCENARIO_H */

