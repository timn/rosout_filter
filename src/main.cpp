
/***************************************************************************
 *  main.cpp - rosout filter main app
 *
 *  Created: Sat Jul 02 15:58:00 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *             2011  SRI International
 *             2011  Carnegie Mellon University
 *             2011  Intel Labs Pittsburgh
 *             2011  Columbia University in the City of New York
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the 3-clase BSD License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE file in the root directory.
 */

#include "rosout_filter.h"
#include <ros/ros.h>

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "rosspawn");
  ros::NodeHandle n;
 
  ROSoutFilter rosout_filter(n, true);

  ros::spin();
}
