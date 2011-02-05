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

#ifndef VEHICLE_TYPE_H
#define VEHICLE_TYPE_H

#include "ns3/object.h"

using namespace ns3;

class VehicleType : public Object {
public:
  static TypeId GetTypeId();
  VehicleType(double length=0, double width=0, double height=0, std::string name="");
  virtual ~VehicleType();
  
  double GetLength() const;
  double GetWidth() const;
  double GetHeight() const;
  std::string GetName() const;
  
  void SetLength(double length);
  void SetWidth(double width);
  void SetHeight(double height);
  void SetName(std::string name);

private:  
  double m_length;
  double m_width;
  double m_height;
  std::string m_name;
};

#endif // VEHICLE_TYPE_H
