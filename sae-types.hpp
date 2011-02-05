#ifndef SAE_TYPES_H
#define SAE_TYPES_H

#include <stdint.h>
#include <ostream>
#include "cpp/Math.hpp"

namespace sae {
  
/** Accuracy in 1=0.01 m */
struct Position {
  /** Sets the position in units of 0.01 meter */
  Position(int32_t x_=0, int32_t y_=0) : x(x_), y(y_) {}
  double GetXinMeters() const { return (double)x / CF; }
  double GetYinMeters() const { return (double)y / CF; }
  static const double CF = 100.0f;;
  static const uint16_t SIZE = 8;
  /** Sets the position in meter */
  static Position InMeters(double x_, double y_) {
    return Position((int32_t)(x_ * CF), (int32_t)(y_ * CF));
  }
  int32_t x, y;
  //short z; //altitude
};

/** Units of 0.01 m/s */
struct Speed {
  /** set the speed in units of 0.01 m/s */
  explicit Speed(int16_t s = 0) : speed(s) {}
  double GetMetersPerSecond() const { return (double)speed * 0.01f; }
  static const uint16_t SIZE = 2;
  /** set the speed im m/s */
  static Speed InMetersPerSecond(double s) {
    return Speed((int16_t)(s*100));
  }
  int16_t speed;
};

/**
 *  The current heading of the vehicle, expressed in unsigned units of 0.010986328 degrees from North 
 * (such that 32767 such degrees represent 359.98900 degrees).  North shall be defined as the axis defined by 
 * the  WSG-84 coordinate system and its reference ellipsoid.  Headings "to the east" are defined as the 
 * positive direction.  A 2 byte value. 
 * Heading  ::=  INTEGER  (0..32767) -- LSB of 0.010986328 degrees 
 */
struct Heading {
  /** Set the heading in units of 0.010986328 degrees from North */
  explicit Heading(int16_t h = 0) : heading(h) {}
  /** Set the heading in degrees from North */
  static Heading InDegrees(double deg) {
    return Heading((int16_t)(deg/CF));
  }
  static Heading InRadians(double rad) {
    double deg = rad * 180 / cpp::math::PI;
    return Heading((int16_t)(deg/CF));
  }
  double GetDegrees() const { return (double)heading * CF; }
  double GetRadians() const { return (double)heading * CF * cpp::math::PI / 180; }
  static const uint16_t SIZE = 2;
  static const double CF = 0.010986328f; //conversion factor;
  int16_t heading;
};

/**
 * A data element representing the signed acceleration of the vehicle along some known axis in units of 
 * 0.01 meters per second squared. A 2 byte long value when sent.  
 * Acceleration ::= INTEGER (-2000..2000) -- LSB units are 0.01 m/s^2 
 */
struct Acceleration {
  /** Sets the acceleration in units of 0.01 m/s^2 */
  explicit Acceleration(int16_t a = 0) : acceleration(a) {}
  
  /** Sets the acceleration in m/s^2 */
  static Acceleration InMetersPerSecondSquared(double a) {
    return Acceleration((int16_t)(a/0.01f));
  }
  double GetMetersPerSecondSquared() const { return (double) acceleration * 0.01f;}
  static const uint16_t SIZE = 2;
  int16_t acceleration;
};

/** Units in 0.01 meter */
struct VehicleSize {
  /** Sets the size in units of 0.01 meter */
  VehicleSize(int16_t l=0, int16_t w=0) : length(l), width(w) {}
  /** Sets the size in meter */
  static VehicleSize InMeters(double l, double w) {
    return VehicleSize((int16_t)(l * CF), (int16_t)(w * CF));
  }
  double GetLengthInMeters() const { return (double)length / CF; }
  double GetWidthInMeters() const { return (double)width / CF; }
  static const double CF = 100.0f;;
  static const uint16_t SIZE = 4;
  int16_t length;
  int16_t width;
};

/** Vehicle kinematics state */
struct VehicleState {
  VehicleState(Position pos = Position(), Speed s = Speed(), Heading h = Heading(), 
               Acceleration a = Acceleration(), VehicleSize vsize = VehicleSize()) : 
      position(pos), speed(s), heading(h), acceleration(a), size(vsize) {}
  
  VehicleState(double x_, double y_, double speed_, double headingRad_, double acceleration_, double vlength_, double vwidth_) {
    position = Position::InMeters(x_,y_);
    speed = Speed::InMetersPerSecond(speed_);
    heading = Heading::InRadians(headingRad_);
    acceleration = Acceleration::InMetersPerSecondSquared(acceleration_);
    size = VehicleSize::InMeters(vlength_, vwidth_);
  }
  static const uint16_t SIZE = Position::SIZE + Speed::SIZE + Heading::SIZE + Acceleration::SIZE + VehicleSize::SIZE;
  Position position;
  Speed speed;
  Heading heading;
  Acceleration acceleration;
  VehicleSize size;
  //friend std::ostream& operator <<(std::ostream &os,const VehicleState &obj);
};

inline std::ostream& operator <<(std::ostream &os, const sae::VehicleState &obj) {
  os << "[VehicleState: pos(m)=" << obj.position.GetXinMeters() << "," << obj.position.GetYinMeters();
  os << " speed(m/s)=" << obj.speed.GetMetersPerSecond() << " heading(d)=" << obj.heading.GetDegrees();
  os << " acceleration(m/s2)=" << obj.acceleration.GetMetersPerSecondSquared();
  os << " size(m)=" << obj.size.GetLengthInMeters() << "x"<<obj.size.GetWidthInMeters();
  return os;
}

}


#endif /* SAE_TYPES_H */
