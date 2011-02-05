#ifndef NS3_VECTOR_UTILS_H
#define NS3_VECTOR_UTILS_H

#include "ns3/vector.h"
#include <cmath>

namespace ns3 {
  const Vector3D Vector3DValueZero;
  
/*  bool operator== (const Vector3D &v1, const Vector3D &v2) {
    return (v1.x == v2.x) && (v1.y == v2.y) && (v1.z == v2.z);
  } 
  
  bool operator!= (const Vector3D &v1, const Vector3D &v2) {
    return !(v1 == v2);
  } */
  
  inline const Vector3D operator+ (const Vector3D &v1, const Vector3D &v2) {
    Vector3D result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
  }
  inline const Vector3D operator- (const Vector3D &v1, const Vector3D &v2) {
    Vector3D result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
    return result;
  }
  inline const Vector3D operator* (const Vector3D &vector, const double& scalar) {
    Vector3D result;
    result.x = vector.x * scalar;
    result.y = vector.y * scalar;
    result.z = vector.z * scalar;
    return result;
  }
  inline const Vector3D operator* (const double& scalar, const Vector3D &vector) {
    return vector * scalar;
  }
  
  inline double CalculateDotProduct(const Vector3D &v1, const Vector3D &v2) {
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
  }
  inline Vector3D CalculateCrossProduct(const Vector3D &v1, const Vector3D &v2) {
    Vector3D v3;
    v3.x = v1.y * v2.z - v1.z * v2.y;
    v3.y = v1.z * v2.x - v1.x * v2.z;
    v3.z = v1.x * v2.y - v1.y * v2.x;
    return v3;
  }
  /** Calculate cross product */
  inline const Vector3D operator* (const Vector3D &v1, const Vector3D &v2) {
    return CalculateCrossProduct(v1, v2);
  }
  
  inline double CalculateLength (const Vector3D& v) {
    double length = std::sqrt (v.x * v.x + v.y * v.y + v.z * v.z);
    return length;
    //return CalculateDistance(Vector3DValueZero, v);
  }

  inline Vector3D Normalize (const Vector3D& v) {
    return v * (1.0f / CalculateLength(v));
  }
  
  inline bool Equals(const Vector3D &v1, const Vector3D &v2, float precision = 0.0001f) {
    return (fabs(v1.x - v2.x) < precision) && (fabs(v1.y - v2.y) < precision) && (fabs(v1.z - v2.z) < precision); 
  }

  inline double CalculateDistanceSquare(const Vector& v1, const Vector& v2) {
    double dx = v1.x - v2.x;
    double dy = v1.y - v2.y;
    double dz = v1.z - v2.z;
    return dx*dx + dy*dy + dz*dz;
  }

}

#endif /* NS3_VECTOR_H */
