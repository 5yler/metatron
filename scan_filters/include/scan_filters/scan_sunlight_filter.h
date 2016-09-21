/********************************************************************
  Software License Agreement (BSD License)

  Copyright (c) 2016, Syler Wagner.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/**
 * scan_sunlight_filter.h
 * LaserScan filter which removes random noise caused by sunlight.
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2016-09-19    creation
 **/

#ifndef LASER_SCAN_SUNLIGHT_FILTER_H
#define LASER_SCAN_SUNLIGHT_FILTER_H

#include <set>

#include "filters/filter_base.h"
#include <sensor_msgs/LaserScan.h>

namespace scan_filters {

  class LaserScanSunlightFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
  public:

    double tolerance_;  //$ distance tolerance
    int window_;        //$ number of adjacent points to check

    LaserScanSunlightFilter() {}          //$ constructor
    virtual ~LaserScanSunlightFilter() {} //$ destructor

    /*$
     * Configure filter parameters for tolerance and window.
     */
    bool configure()
    {
      window_ = 2;  //$ default value
      if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("tolerance"), tolerance_))
      {
        ROS_INFO("Error: SunlightFilter was not given tolerance parameter.\n");
        return false;
      }
      if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("window"), window_))
      {
        ROS_INFO("Error: SunlightFilter was not given window parameter.\n");
      }
      return true;
    }

    /*$
     * Check if a range value value is valid.
     */
    inline bool checkRange(double r, double r_max, double r_min)
    {
      if ((r < r_min) || (r > r_max) || std::isnan(r))
      {
        return false;
      }
      return true;
    }


    /*$ 
     * Remove invalid points.
     * \param scan_in the input LaserScan message
     * \param scan_out the output LaserScan message
     */
    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
    {
      scan_out = scan_in; //$ copy data

      std::set<int> indices_to_delete;  //$ keep track of which indices need to be removed

      //$ iterate over all points in scan
      for (unsigned int i = 0; i < scan_in.ranges.size(); i++)
      {
        //$ delete a point if it's invalid
        if (checkRange(scan_in.ranges[i], scan_in.range_max, scan_in.range_min) == false) 
        {
          indices_to_delete.insert(i);
        }
        else
        {
          bool neighbor_match = false;

          //$ check neighboring points
          for (int y = -window_; y < window_ + 1; y++)
          {
            int j = i + y;  //$ neighboring point index

            //$ this if statement is triggered when out of list bounds or trying to compare to self
            if ( j < 0 || j >= (int)scan_in.ranges.size() || (int)i == j )
            { 
              continue; 
            }

            //$ if neighboring point is OK check distance to point
            if (checkRange(scan_in.ranges[j], scan_in.range_max, scan_in.range_min) && (fabs(scan_in.ranges[j] - scan_in.ranges[i]) < tolerance_))
            {
              neighbor_match = true; //$ at least one valid neighbor
              break;
            }
          } //$ end for (int y = -window_; y < window_ + 1; y++)
          
          if (neighbor_match == false) //$ no neighbors with range within tolerance_ of this point
          {
            indices_to_delete.insert(i);
          }
        }
      } //$ end for (unsigned int i = 0; i < scan_in.ranges.size(); i++)

      ROS_INFO("LaserScanSunlightFilter removing %d Points from scan with min tolerance: %.2f and window: %d", (int)indices_to_delete.size(), tolerance_, window_);
      for ( std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
      {
        scan_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();  // Failed test to set the ranges to invalid value
      }
      return true;
    }

  };
}

#endif //$ LASER_SCAN_SUNLIGHT_FILTER_H
