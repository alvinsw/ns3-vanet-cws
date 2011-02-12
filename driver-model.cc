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

#include "driver-model.h"
#include <cmath>
#include "ns3/assert.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("DriverModel");


//DriverModel::Action::Action(double a, DriverModel::LaneChange lc) : acceleration(a), laneChange(lc)
DriverModel::Action::Action() : acceleration(0.0f), laneChange(None), delay(0.0f)
{ }

NS_OBJECT_ENSURE_REGISTERED (DriverModel);
TypeId 
DriverModel::GetTypeId(void) {
  static TypeId tid = TypeId("DriverModel")
    .SetParent<Object> ()
    .AddConstructor<DriverModel> ()
    ;
  return tid;
}
DriverModel::DriverModel() { }
DriverModel::~DriverModel() { }

bool 
DriverModel::CalculateAction(const DriverInput& di, DriverModel::Action& action) const {
  return false;
}

Ptr< RoadTrafficScenario > DriverModel::GetRTS() const { return m_rts; }
void DriverModel::SetRTS(Ptr< RoadTrafficScenario > rts) { 
  m_rts = rts; 
  m_tolerance = rts->pUpdatePeriod.GetSeconds();
}

void DriverModel::DoDispose(void )
{
  m_rts = 0;
  Object::DoDispose();
}


NS_OBJECT_ENSURE_REGISTERED (ConstantSpeedWithBrakingDriverModel);
TypeId 
ConstantSpeedWithBrakingDriverModel::GetTypeId(void) {
  static TypeId tid = TypeId("ConstantSpeedWithBrakingDriverModel")
    .SetParent<DriverModel> ()
    .AddConstructor<ConstantSpeedWithBrakingDriverModel> ()
    ;
  return tid;
}

ConstantSpeedWithBrakingDriverModel::ConstantSpeedWithBrakingDriverModel(double minGap_, double maxDecel_) :
    DriverModel(), minGap(minGap_), maxDecel(maxDecel_)
{ }

bool ConstantSpeedWithBrakingDriverModel::CalculateAction (const DriverInput& di, DriverModel::Action& action) const {
  //NS_LOG_DEBUG ("ConstantSpeedWithBrakingDriverModel::CalculateAction ");
  double B_a = di.GetVehicleAcceleration();
  double B_v = di.GetVehicleSpeed();
  if (B_a <= -maxDecel) return false;
  if (B_v == 0.0f) return false;
  DriverInput::VehicleState F; //front
  if (di.GetLeadingVehicle(F)) {
    double F_a = F.acceleration;
    double F_v = F.speed;
    action.delay = 0.0f;
    action.laneChange = None;
    double rt = di.GetReactionTime().GetSeconds();
    if (F_a < 0.0f || F_v == 0.0f) { // a case where stopping point is predictable: leader is decelerating or not moving
      //find stopping point
      double F_stopDistance = 0;
      if (F_a != 0.0f) F_stopDistance = -0.5 * F_v * F_v / F_a;
      double B_v_rt = B_v + B_a * rt;
      if (B_v_rt <= 0.0f) return false;
      double B_s_rt = B_v * rt + 0.5*B_a*rt*rt;
      double B_stopDistance = B_s_rt + 0.5*B_v_rt*B_v_rt / maxDecel;
      //NS_LOG_FUNCTION (this << B_v_rt << B_s_rt << F.gap << F_stopDistance << B_stopDistance << minGap);
      //NS_LOG_DEBUG("total="<<F.gap + F_stopDistance - B_stopDistance - minGap);
      double toleranceDist = m_tolerance * B_v;
      double dd = F.gap + F_stopDistance - B_stopDistance - minGap;
      if ( dd - toleranceDist > 0.0f) {
        return false;
      } else {
        //check scheduled course change
        if (di.GetProjectedVehicleTime().IsStrictlyPositive()) {
          double a = di.GetProjectedVehicleAcceleration();
          if (a == -maxDecel) {
            return false;
          }
          if (a < 0.0f) {
            double v = di.GetProjectedVehicleSpeed();
            double projStopOffset = di.GetProjectedVehicleOffset() + 0.5*v*v / (-a);
            //double result = F.gap + F_stopDistance - (projStopOffset - di.GetVehicleOffset());
            //NS_LOG_FUNCTION (this << F.gap << F_stopDistance << projStopOffset << di.GetVehicleOffset());
            //NS_LOG_DEBUG ("1 result="<<result);
            if (F.gap + F_stopDistance - (projStopOffset - di.GetVehicleOffset()) > 0.0f) return false;
          }
        }
        if (dd > 0.0f && B_v > 0.0f) action.delay = dd / B_v;
        action.acceleration = -maxDecel;
        return true;
      }
    } else { //leader is accelerating or constant speed
      //double da = F_a - B_a;
      //if (std::abs(da) < 0.000001f) da = 0.0f;
      double dv0 = B_v - F_v;
      //if (std::abs(dv0) < 0.000001f) dv0 = 0.0f;
      if (dv0 > 0.0f) {
        //ttc=
        double TTC = (F.gap - minGap) / dv0;
        double TTA = (dv0 / maxDecel) + rt;
        if (TTC - TTA < 0.0f) {
          if (di.GetProjectedVehicleAcceleration() > (-maxDecel)) {
            action.acceleration = -maxDecel;
            return true;
          }
        }
      }
    }
  }
  return false;
}


NS_OBJECT_ENSURE_REGISTERED (IdmDriverModel);
TypeId 
IdmDriverModel::GetTypeId(void) {
  static TypeId tid = TypeId("IdmDriverModel")
    .SetParent<DriverModel> ()
    .AddConstructor<IdmDriverModel> ()
    ;
  return tid;
}

IdmDriverModel::IdmDriverModel(double desiredSpeed, double timeHeadway, double minimumGap, double maxAccel, double brakeDecel, double maxDecel) :
    v0(desiredSpeed), T(timeHeadway), s0(minimumGap), a(maxAccel), b(brakeDecel), max_b(maxDecel), a_exp(4)
{ }

bool 
IdmDriverModel::CalculateAction(const DriverInput& di, DriverModel::Action& action) const {
  double v = di.GetVehicleSpeed();
  double dv = 0.0f;
  double s = 0.0f;
  DriverInput::VehicleState F;
  if (di.GetLeadingVehicle(F)) {
    dv = v - F.speed;
    s = F.gap;
  }
  action.acceleration = CalculateAcceleration(v, dv, s);
  action.laneChange = None;
  action.delay = 0;
  return true;
}

/** gap == 0 means that no leading vehicle (will return acceleration as in free road) */
double 
IdmDriverModel::CalculateAcceleration(double v, double dv, double s) const {
  double brakingStrategy = 0;
  if (s > 0.0f) {
    double desiredMinGap = s0 + v*T + ((v*dv)/(2*std::sqrt(a*b)));
    brakingStrategy = desiredMinGap / s;
    brakingStrategy *= brakingStrategy; // brakingStrategy = (desiredMinGap / s) ^2
  }
  double dvpdt = a * ( 1 - std::pow(v/v0, a_exp) - brakingStrategy);
  if (dvpdt > max_b) dvpdt = max_b;
  return dvpdt;
}

NS_OBJECT_ENSURE_REGISTERED (MobilDriverModel);
TypeId 
MobilDriverModel::GetTypeId(void) {
  static TypeId tid = TypeId("MobilDriverModel")
    .SetParent<DriverModel> ()
    .AddConstructor<MobilDriverModel> ()
    ;
  return tid;
}

MobilDriverModel::MobilDriverModel(const IdmDriverModel& idmRef, double politenessFactor, double maximumSafeDeceleration, double threshold) :
    IdmDriverModel(idmRef), p(politenessFactor), b_save(maximumSafeDeceleration), a_thr(threshold)
{ }


MobilDriverModel::MobilDriverModel(double politenessFactor, double maximumSafeDeceleration, double threshold) :
    IdmDriverModel(), p(politenessFactor), b_save(maximumSafeDeceleration), a_thr(threshold) 
{ }

bool 
MobilDriverModel::CalculateAction(const DriverInput& di, DriverModel::Action& action) const {
  LaneChange lc = None;
  double accM = 0.0f;
  double Mv = di.GetVehicleSpeed();
  double Ml = di.GetVehicleLength();
  DriverInput::VehicleState F;
  if (di.GetLeadingVehicle(F)) {
    accM = CalculateAcceleration(Mv, Mv - F.speed, F.gap);
    
    if (di.AllowsLeftLaneChange() || di.AllowsRightLaneChange()) {
      double incentive = 0.0f;
      double accB = 0.0f;
      double acc1B = 0.0f;
      DriverInput::VehicleState B;
      if (di.GetFollowingVehicle(B)) {
        accB = CalculateAcceleration(B.speed, B.speed - Mv, B.gap);
        acc1B = CalculateAcceleration(B.speed, B.speed - F.speed, B.gap + F.gap + Ml);
      }
      if (di.AllowsLeftLaneChange()) {
        DriverInput::VehicleState B1;
        DriverInput::VehicleState F1;
        std::pair<bool,bool> r = di.GetLeftLaneVehicles(B1, F1);
        incentive = CalculateIncentive(r.first, B1, r.second, F1, Mv, Ml, accM, accB, acc1B);
        if (incentive > 0.0f) {
          lc = Left;
        }
      }
      if (di.AllowsRightLaneChange()) {
        DriverInput::VehicleState B1;
        DriverInput::VehicleState F1;
        std::pair<bool,bool> r = di.GetRightLaneVehicles(B1, F1);
        double temp_incentive = CalculateIncentive(r.first, B1, r.second, F1, Mv, Ml, accM, accB, acc1B);
        if (temp_incentive > incentive) {
          lc = Right;
        }
      }
    }
    
  } else {
    accM = CalculateAcceleration(Mv, 0.0f, 0.0f);
  }
  action.acceleration = accM;
  action.laneChange = lc;
  action.delay = 0;
  return true;
}

double MobilDriverModel::CalculateIncentive(bool backExist, const DriverInput::VehicleState& B1, bool frontExist, const DriverInput::VehicleState& F1, double Mv, double Ml, double accM, double accB, double acc1B) const {
  double incentive = 0.0f;
  if ((backExist && B1.gap > 0.0f) || (frontExist && F1.gap > 0.0f)) {
    double acc1B1 = 0.0f;
    if (backExist) {
      acc1B1 = CalculateAcceleration(B1.speed, B1.speed - Mv, B1.gap);
    }
    if (acc1B1 > -b_save) {
      double accB1 = 0.0f;
      double acc1M1 = 0.0f;
      if (frontExist) {
        if (backExist) accB1 = CalculateAcceleration(B1.speed, B1.speed - F1.speed, B1.gap + F1.gap + Ml);
        acc1M1 = CalculateAcceleration(Mv, Mv - F1.speed, F1.gap);
      } else {
        if (backExist) accB1 = CalculateAcceleration(B1.speed);
        acc1M1 = CalculateAcceleration(Mv);
      }
      incentive = (acc1M1 - accM) - (p*(accB + accB1 - acc1B - acc1B1) + a_thr);
    }
  }
  return incentive;
}


NS_OBJECT_ENSURE_REGISTERED (SimpleCwsDriverModel);
TypeId 
SimpleCwsDriverModel::GetTypeId(void) {
  static TypeId tid = TypeId("SimpleCwsDriverModel")
    .SetParent<DriverModel> ()
    .AddConstructor<SimpleCwsDriverModel> ()
    ;
  return tid;
}

SimpleCwsDriverModel::SimpleCwsDriverModel(double minGap_, double maxDecel_) :
    DriverModel(), minGap(minGap_), maxDecel(maxDecel_)
{ }

bool SimpleCwsDriverModel::CalculateAction(const DriverInput& di, DriverModel::Action& action) const {
  double v0 = di.GetVehicleSpeed();
  //double rDist = v0 * reactionTime;
  DriverInput::VehicleState L;
  if (di.GetLeadingVehicle(L)) {
    //std::cout << "[SimpleCwsDriverModel::CalculateAction] L.nodeId=" << L.nodeId <<std::endl;
    if (L.acceleration < 0 || L.speed < v0) {
    double t = (v0 - L.speed) / maxDecel;
    double reactionTime = 1.0f;
    double brakingDist = v0 * reactionTime + 0.5 * maxDecel * t * t;
    action.laneChange = None;
    action.delay = 0;
    if ((L.gap - brakingDist - minGap) < 0) { //collision imminent
      action.acceleration = -maxDecel;
      return true;
    } else { // safe
      if (std::abs(di.GetVehicleAcceleration()-action.acceleration) < 0.001f) {
        return false;
      } else {
        action.acceleration = 0;
        return true;
      }
    }
      
    }
  }
  return false;
}



