/*
 *  gps_common.cpp
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <kybernetes/math/gps_common.hpp>
#include <cmath>

using namespace kybernetes::math;

// Convert degrees to radians
double toRad(double deg)
{
    return deg * (M_PI / 180.0);
}

// Convert radians to degrees
double toDeg(double rad)
{
    return rad * (180.0 / M_PI);
}


// Default constructor
GeoCoordinate::GeoCoordinate()
{
    latitude = 0.0;
    longitude = 0.0;
}

// Constructor, just copy in latitude and longitude and be done
GeoCoordinate::GeoCoordinate(double _latitude, double _longitude)
    : latitude(_latitude), longitude(_longitude)
{
    
}

// Get the forward azimuth to another gps coordinate
double GeoCoordinate::headingTo(GeoCoordinate &coordinate)
{
    // Convert the bearing to radians
    double rlat1 = toRad(latitude);
    double rlng1 = toRad(longitude);
    double rlat2 = toRad(coordinate.latitude);
    double rlng2 = toRad(coordinate.longitude);
    
    // Perform the bearing calculation
    double dLon = rlng2 - rlng1;
    double y = std::sin(dLon) * std::cos(rlat2);
    double x = (std::cos(rlat1) * std::sin(rlat2)) - (std::sin(rlat1) * std::cos(rlat2) * std::cos(dLon));
    
    // Return the bearing
    return toDeg(std::atan2(y, x));
}

// Get the distance in meters to another gps coordinate
double GeoCoordinate::distanceTo(GeoCoordinate &coordinate)
{
    // Convert the bearing to radians
    double R     = 6371000.0;
    double rlat1 = toRad(latitude);
    double rlat2 = toRad(coordinate.latitude);
    double dLat  = toRad(coordinate.latitude - latitude);
    double dLon  = toRad(coordinate.longitude - longitude);
    
    // Perform great circle calculations
    double s1 = std::sin(dLat/2);
    double s2 = std::sin(dLon/2);
    double a = (s1 * s1) + (s2 * s2 * std::cos(rlat1) * std::cos(rlat2));
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c;
}