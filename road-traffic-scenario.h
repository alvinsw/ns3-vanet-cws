#ifndef ROAD_TRAFFIC_SCENARIO_H
#define ROAD_TRAFFIC_SCENARIO_H

#include "ns3/node-container.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "vehicle-mobility-model.h"
#include "vehicle-type.h"
#include <memory>

class DriverInput;
class DriverModel;

class RoadTrafficScenario : public SimpleRefCount<RoadTrafficScenario> {
  public:
    typedef Callback< void, Ptr<Node>, double, double > NotifyAccelerationChange; // node, old accel, new accel
    typedef Callback< void, Ptr<Node>, double, double > NotifyCollision; //node, crashedAtSpeed, speedDiff
    //typedef Callback< void, Ptr<Node> > NotifyInactiveNode; //node
    
    enum Direction { LEFT, RIGHT };
    
    struct AccelChangeEvent {
      AccelChangeEvent(Time time_ = Seconds(0), uint32_t laneId_=0, uint32_t indexAtLane_=0, float accel_=0.0f);
      Time  time;
      uint32_t laneId;
      uint32_t indexAtLane;
      float accel;
    };
    
    struct VehicleState {
      VehicleState(double position_=0, float speed_=0, float acceleration_=0, Ptr<VehicleType> vehicleType_=0);
      //Vector position;
      double position;
      float speed;
      float acceleration;
      Ptr<VehicleType> vehicleType;
      //uint32_t laneId;
      //uint32_t indexAtLane;
    };
    
    class Lane {
    public:
      Lane(Ptr<LanePath> p=0);
      ~Lane();
      //~Lane();
      //uint32_t GetLastVehicleIndex() const;
      //uint32_t laneId;
      Ptr<LanePath> path;
      std::vector<VehicleState> initialStates;
      //std::vector< Ptr<Node> > vehicles;
    };
    
    struct NodeData {
      NodeData();
      VehicleState projectedState;
      Time projectedTime;
      bool isManual;
    };
    
    //static double defaultDistanceEpsilon;
    static bool defaultStopAtEndPoint;
    
    RoadTrafficScenario();
    virtual ~RoadTrafficScenario();
    
    /** Number of initial vehicles to be prepared for simulation. */
    virtual uint32_t ExpectedVehiclesCount() const;
    
    /** Number of vehicles ready to run in simulation. */
    uint32_t VehiclesCount() const;
    
    /** If true, the simulation will be stopped after there is no moving vehicle. Default is true. */
    void AllowsStopSimulation(bool val);
    
    /** Smart braking is enabled by default. */
    void EnablesSmartBraking(bool val = true);
    /** min < max; If min != max then will use random value between them inclusive. */
    void SetReactionTime(Time min, Time max);
    void SetReactionTime(Time rt);
    void SetNotifyAccelerationChange( NotifyAccelerationChange callback);
    void SetNotifyCollision( NotifyCollision callback);
    void SetNotifyActiveStatusChange( VehicleMobilityModel::ActiveStatusChangeCallback callback);
    void SetDriverInput(Ptr<DriverInput> di);
    void SetDriverModel(Ptr<DriverModel> dm);
    
    /** 
     * Pre-set an accident event before Install. An accident is implemented by 
     * decelerating a vehicle using the vehicleAbnormalDecel parameter.
     */
    void PreSetAccident(Time accidentTime, uint32_t laneId, uint32_t indexAtLane);
    
    /** Pre-set a braking event before Install. */
    void PreSetAccelerationChange(Time t, uint32_t laneId, uint32_t indexAtLane, float a);
    //void CreateLanes(uint32_t lanesNum);
    Lane& AddLane(Ptr<LanePath> lanePath);
    Lane& GetLane(uint32_t laneId);
    const Lane& GetLane(uint32_t laneId) const;
    virtual uint32_t GetNextLaneId(uint32_t laneId, Direction side) const = 0;
    uint32_t LanesCount() const;
    Time CalculateReactionTime(uint32_t nodeId) const;
    
    /** Setup given nodes with this scenario. Call this method after setting up everything else. */
    void Install(NodeContainer& nodes);
    /** Call this method after the simulation finish. */
    virtual void Dispose();
    
    //The following methods must be called after Install
    
    /** 
     * The specified node will perform braking after reaction time, only if it is necessary.
     * If smart braking is disabled, perform braking after reaction time with a deterministic deceleration
     */
    void InitiateBraking(Ptr< Node > node);
    void ScheduleEmergencyBraking(Ptr<Node> node, Time t);
    void ScheduleNormalBraking(Ptr<Node> node, Time t);
    Ptr<Node> GetFollower(Ptr<Node> node) const;
    Ptr<Node> GetLeader(Ptr<Node> node) const;
    
    ////** Estimated Collision distance in meters */
    //double EstimateCollisionDistance(Ptr<VehicleMobilityModel> m) const;
    
    /////////////// PUBLIC PARAMETERS
    std::vector< AccelChangeEvent > pAccelChangeEvents; //laneId, indexAtLane
    //float pVehicleSpeed;
    float pVehicleAbnormalDecel;
    float pVehicleNormalDecel;
    Time pReactionTimeMin; //reaction time in seconds
    Time pReactionTimeMax;
    float pMinGap;
    Time pUpdatePeriod;
    
  protected:
    virtual uint32_t Setup() = 0;
    /** Returns the last nodeId assigned */
    uint32_t SetupLane(Lane& lane, uint32_t startingNodeId, NodeContainer& nodes);
    /** Returns the number of active vehicles after update */
    uint32_t UpdateLane(Lane& lane);

    /** Finds a Node Id from lane id and index at lane */
    uint32_t FindNodeId(uint32_t laneId, uint32_t indexAtLane) const;
    
    /** Change acceleration immediately and notify the listener. */
    void ChangeAcceleration(Ptr<Node> node, double newAcceleration);
    
    /** Schedule acceleration change after the specified time t. */
    void ScheduleChangeAcceleration(Ptr<Node> node, double newAcceleration, Time t);
    
    /** 
     * The braking deceleration will be adjusted based on the vehicle state 
     * in such a way so that the vehicle will stop just close to the leader if possible
     */
    void InitiateSmartBraking(Ptr< Node > node);
    
  private:
    void Update();

    //TracedCallback<Ptr<const Node> > m_decelerationChangeTrace;
    /** Used to alert subscribers that a change in acceleration has occurred. */
    NotifyAccelerationChange m_notifyAccelerationChange;
    /** Alert subscribers that a collision has occurred. */
    NotifyCollision m_notifyCollision;
    /** Alert subscribers when a node become inactive (out of the simulation area). */
    VehicleMobilityModel::ActiveStatusChangeCallback m_notifyActiveStatusChange;
    std::vector< Lane > m_lanes;
    uint32_t m_vehiclesCount;
    bool m_stopSimulationAllowed;
    bool m_smartBrakingEnabled;
    bool m_stopAtEndPoint;
    //std::vector<double> m_smartBrakingPredictedStopOffset;
    //mutable std::vector<double> m_collisionDistance;
    //std::vector<bool> m_isManual;
    
    Ptr<DriverInput> m_driverInput;
    Ptr<DriverModel> m_driverModel;
    
    std::vector<NodeData> m_data;
    
};

#endif /* ROAD_TRAFFIC_SCENARIO_H */
