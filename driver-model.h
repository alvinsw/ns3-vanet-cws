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

#ifndef DRIVER_MODEL_H
#define DRIVER_MODEL_H

#include "ns3/node.h"
#include "vehicle-mobility-model.h"
#include "driver-input.h"
#include "road-traffic-scenario.h"
#include "default-parameters.h"
/**
 * Do nothing driver model. Define a custom driver behaviour by subclassing this class.
 */
class DriverModel : public Object {
public:
  enum LaneChange { None, Right, Left };
  struct Action {
    Action();
    //explicit Action(double a = 0.0f, LaneChange lc = None, double delay_=0.0f);
    double acceleration;
    LaneChange laneChange;
    double delay;
  };
  static TypeId GetTypeId(void);
  
  Ptr<RoadTrafficScenario> GetRTS() const;
  void SetRTS(Ptr<RoadTrafficScenario> rts);
  
  DriverModel();
  virtual ~DriverModel();
  virtual bool CalculateAction(const DriverInput& di, Action& action) const;

protected:
  virtual void DoDispose(void );
  // update period in seconds
  double m_tolerance;
    
private:
  Ptr<RoadTrafficScenario> m_rts;
};

class ConstantSpeedWithBrakingDriverModel : public DriverModel {
  public:
    static TypeId GetTypeId(void);
    ConstantSpeedWithBrakingDriverModel(double minGap_ = DP_VEH_MIN_GAP, double maxDecel_ = DP_VEH_DECEL);
    virtual ~ConstantSpeedWithBrakingDriverModel() {}
    virtual bool CalculateAction(const DriverInput& di, Action& action) const;
  protected:
    double minGap;
    double maxDecel;
    //double reactionTime;
};

class IdmDriverModel : public DriverModel {
  public:
    static TypeId GetTypeId(void);
    IdmDriverModel(double desiredSpeed = 33.333f, double timeHeadway = 1.5f, double minimumGap = 2.0f, double maxAccel = 1.0f, double brakeDecel = 2.0f, double maxDecel = 4.9f);
    virtual ~IdmDriverModel() {}
    virtual bool CalculateAction(const DriverInput& di, Action& action) const;
    double CalculateAcceleration(double v, double dv = 0.0f, double s = 0.0f) const;
  protected:
    double v0; //desiredVelocity
    double T;//timeHeadway
    double s0; //minimum gap
    double a; //maxAcceleration
    double b; //comfortableBrakingDeceleration
    double max_b; //maximumBrakingDeceleration
    double a_exp; //accelerationExponent
};

class MobilDriverModel : public IdmDriverModel {
  public:
    static TypeId GetTypeId(void);
    /** Pass a reference to idm object that must be guaranteed to be valid for this object life time. */
    MobilDriverModel(const IdmDriverModel& idmRef, double politenessFactor = 0.5f, double maximumSafeDeceleration = 4.0f, double threshold = 0.2f);
    MobilDriverModel(double politenessFactor = 0.5f, double maximumSafeDeceleration = 4.0f, double threshold = 0.2f);
    virtual ~MobilDriverModel() {}
    virtual bool CalculateAction(const DriverInput& di, Action& action) const;
  protected:
    double p; //politenessFactor
    double b_save; //MaximumSafeDeceleration
    double a_thr; //threshold
  private:
    double CalculateIncentive(bool backExist, const DriverInput::VehicleState& B1, bool frontExist, const DriverInput::VehicleState& F1, double Mv, double Ml, double accM, double accB, double acc1B) const;
};

/** Avoid collision at the last moment*/
class SimpleCwsDriverModel : public DriverModel {
  public:
    static TypeId GetTypeId(void);
    SimpleCwsDriverModel(double minGap_ = 2.0f, double maxDecel_ = 4.9f);
    virtual ~SimpleCwsDriverModel() {}
    virtual bool CalculateAction(const DriverInput& di, Action& action) const;
    //double CalculateAcceleration(double v, double dv = 0.0f, double s = 0.0f) const;
  protected:
    double minGap;
    double maxDecel;
};


#endif // DRIVER_MODEL_H

