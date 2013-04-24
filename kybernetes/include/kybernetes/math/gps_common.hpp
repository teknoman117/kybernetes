/*
 *  gps_common.hpp
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

#ifndef _kybernetes_sensor_gps_common_h_
#define _kybernetes_sensor_gps_common_h_

// Kybernetes namespace
namespace kybernetes
{
    // controller namespace
    namespace math
    {
        // class that manages the imu
        class GeoCoordinate
        {
        public:
            // Standard constructor
            GeoCoordinate();
            GeoCoordinate(double _latitude, double _longitude);
            
            // Latitude and longitude of a geocoordinate
            double latitude;
            double longitude;
            
            // Utilities for GPS navigation
            double headingTo(GeoCoordinate& coordinate);
            double distanceTo(GeoCoordinate& coordinate);
        };
    }
}

#endif