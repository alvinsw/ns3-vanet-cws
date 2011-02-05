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

#ifndef LANE_PATH_H
#define LANE_PATH_H

#include "ns3/vector.h"
#include <vector>
#include "vehicle-mobility-model.h"

using namespace ns3;

class LanePath : public SimpleRefCount<LanePath> {
public:
  LanePath();
  virtual ~LanePath();
  /** Calculates a point with the offset distance from the starting point of this path */
  virtual Vector CalculatePoint(double offset) const = 0;
  //** Calculates a point along the path given a starting point, speed, and acceleration */
  //virtual Vector CalculatePoint(Vector startPoint, double offset) const = 0;
  virtual Vector GetDirection(double offset) const = 0;
  virtual double GetLength() const = 0;
  virtual std::string ToString() const = 0;
  void AddFront(Ptr<VehicleMobilityModel> m);
  void AddBack(Ptr<VehicleMobilityModel> m);
  Ptr<VehicleMobilityModel> GetFront() const;
  Ptr<VehicleMobilityModel> GetBack() const;
  uint32_t GetId() const;
  void SetId(uint32_t id);
  void Reset();
  
private:
  //Ptr<LanePath> rightSideLane;
  //Ptr<LanePath> leftSideLane;
  //std::vector< Ptr<LanePath> > nextLanes;
  Ptr<VehicleMobilityModel> front;
  Ptr<VehicleMobilityModel> back;
  uint32_t m_id;
};

class StraightLanePath : public LanePath {
public:
  StraightLanePath();
  StraightLanePath(Vector startPoint_, Vector endPoint_);
  virtual ~StraightLanePath();
  virtual Vector CalculatePoint(double offset) const;
  virtual Vector GetDirection(double offset) const;
  virtual double GetLength() const;
  virtual std::string ToString() const;
private:
  Vector startPoint;
  Vector endPoint;
  Vector direction;
  double distance;
};

#endif // LANE_PATH_H
