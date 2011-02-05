/*
    Vehicle type define a fixed properties of vehicle such as the dimension
    
    Copyright (C) 2010  Alvin Sebastian

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

#include "vehicle-type.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("VehicleType");
NS_OBJECT_ENSURE_REGISTERED (VehicleType);

TypeId 
VehicleType::GetTypeId (void)
{
  static TypeId tid = TypeId ("VehicleType")
    .SetParent<Object> ()
    .AddConstructor<VehicleType>()
    ;
    
  return tid;
}

VehicleType::VehicleType(double length, double width, double height, std::string name) : 
    m_length(length), m_width(width), m_height(height), m_name(name)
{ }
    
VehicleType::~VehicleType () { }

double VehicleType::GetLength() const { return m_length; }

double VehicleType::GetWidth() const { return m_width; }

double VehicleType::GetHeight() const { return m_height; }

std::string VehicleType::GetName() const { return m_name; }


void VehicleType::SetLength(double length) { m_length = length; }

void VehicleType::SetWidth(double width) { m_width = width; }

void VehicleType::SetHeight(double height) { m_height = height; }

void VehicleType::SetName(std::string name) { m_name = name; }

