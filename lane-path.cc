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

#include "lane-path.h"
#include "ns3/log.h"
#include "vector-utils.hpp"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LanePath");


LanePath::LanePath() :  front(), back(), m_id() //rightSideLane(0), leftSideLane(0), nextLanes(1),
{ }

LanePath::~LanePath() { }

uint32_t LanePath::GetId() const
{
  return m_id;
}
void LanePath::SetId(uint32_t id)
{
  m_id = id;
}

void LanePath::AddBack(Ptr< VehicleMobilityModel > m) {
  if (back==0) {
    front = m;
    back = m;
  } else {
    m->SetLeader(back);
    back->SetFollower(m);
    back = m;
  }
}

void LanePath::AddFront(Ptr< VehicleMobilityModel > m) {
  if (front==0) {
    front = m;
    back = m;
  } else {
    m->SetFollower(front);
    front->SetLeader(m);
    front = m;
  }
}

Ptr< VehicleMobilityModel > LanePath::GetFront() const
{
  return front;
}
Ptr< VehicleMobilityModel > LanePath::GetBack() const
{
  return back;
}
void LanePath::Reset()
{
  Ptr<VehicleMobilityModel> m = front;
  while (m!=0) {
    Ptr<VehicleMobilityModel> m1 = m->GetFollower();
    m->SetLeader(0);
    m->SetFollower(0);
    m = m1;
  }
  front = 0;
  back = 0;
  
}


StraightLanePath::StraightLanePath() : LanePath(), startPoint(), endPoint(), direction(), distance(0) 
{ }

StraightLanePath::~StraightLanePath() { }

StraightLanePath::StraightLanePath(Vector startPoint_, Vector endPoint_) : startPoint(startPoint_), endPoint(endPoint_), direction(), distance(0) { 
  direction = Normalize(endPoint - startPoint);
  distance = CalculateDistance(startPoint, endPoint);
}

ns3::Vector StraightLanePath::CalculatePoint(double offset) const {
  return startPoint + (direction * offset);
}

Vector StraightLanePath::GetDirection(double offset) const {
  return direction;
}

double StraightLanePath::GetLength() const {
  return distance;
}

std::string StraightLanePath::ToString() const {
  std::stringstream out;
  out << "[StraightLanePath@" << this << ": startPoint=" << startPoint << ", endPoint=" << endPoint << ", direction=" << direction << "]";
  return out.str();
}

