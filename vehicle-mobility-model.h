#ifndef VEHICLE_MOBILITY_MODEL_H
#define VEHICLE_MOBILITY_MODEL_H

#include "ns3/nstime.h"
#include "ns3/mobility-model.h"
#include "ns3/constant-position-mobility-model.h"

#include "vehicle-type.h"
//#include "lane-path.h"
class LanePath;


/**
 * \brief a position model for which the current speed
 *        follows a vehicle in front of it 
 */
class VehicleMobilityModel : public ConstantPositionMobilityModel {
public:
  static TypeId GetTypeId (void);
  
  /**
   * Create position located at coordinates (0,0,0) with
   * speed (0,0,0) and accelleration (0,0,0).
   */
  VehicleMobilityModel ();
  virtual ~VehicleMobilityModel ();
  
  double GetSpeed() const;
  
  /**
   * \param speed the new speed to set in m/s
   */
  void SetSpeed(double speed);

  /**
   * \param acceleration the new acceleration to set.
   * Negative value means deceleration. Unit is m/s^2
   */
  void SetAcceleration(double acceleration);

  double GetAcceleration() const;
  
  double GetLength() const;
  double GetWidth() const;
  
  uint32_t GetLaneId() const;
  //void SetLaneId(uint32_t id);
  
//   uint32_t GetIndexAtLane() const;
//   void SetIndexAtLane(uint32_t index);
  Ptr<VehicleMobilityModel> GetFollower() const;
  void SetFollower(Ptr<VehicleMobilityModel> m);
  Ptr<VehicleMobilityModel> GetLeader() const;
  void SetLeader(Ptr<VehicleMobilityModel> m);
  
  Ptr<VehicleType> GetVehicleType() const;
  void SetVehicleType(Ptr<VehicleType> vtype);
  
  Ptr<LanePath> GetLanePath() const;
  /** offsetAlongPath will be set to offset, default is 0. */
  void SetLanePath(Ptr<LanePath> lanePath, double offset = 0);
  
  double GetOffsetAlongPath() const;
  void SetOffsetAlongPath(double offset);
  double GetOffsetFrom(Ptr<VehicleMobilityModel> m) const;
  Vector GetDirection() const;
  /** Get heading in radians, north means 0, positive clockwise*/
  double GetHeading() const;
  
  void Update() const;

  //public data member
  bool isCollided;
  //bool isRemoved;
  
  typedef Callback< void, Ptr<VehicleMobilityModel>, bool > ActiveStatusChangeCallback;
  void NotifyActiveStatusChange(bool isActive) const;
  void SetActiveStatusChangeCallback(ActiveStatusChangeCallback callback);
  bool IsActive() const;
  void Activate() const;
  void Deactivate() const;
  
  void NotifyCourseChange(void) const {}
  
  
private:
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;
  
  // private variables
  //uint32_t m_laneId; // Lane Id which this vehicle belong */
  //** position index of a vehicle in its lane */
  //uint32_t m_indexAtLane;
  Ptr<VehicleMobilityModel> m_follower;
  Ptr<VehicleMobilityModel> m_leader;
  mutable  Time m_lastUpdate;
  mutable Vector m_position;
  mutable double m_speed;
  mutable double m_acceleration;
  mutable double m_offsetAlongPath;
  Ptr<VehicleType> m_type;
  Ptr<LanePath> m_lanePath;
  mutable bool m_isActive;
  ActiveStatusChangeCallback m_notifyActiveStatusChange;
  //static double EPSILON = 0.001;
};

#endif /* VEHICLE_MOBILITY_MODEL_H */
