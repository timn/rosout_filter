
/***************************************************************************
 *  rosout_filter.h - rosout filter application
 *
 *  Created: Sat Jul 02 16:00:00 2011
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

#ifndef __ROSOUT_FILTER_H_

#include <ros/common.h>
#if ROS_VERSION_MAJOR > 1 || ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR >= 4
#  include <rosgraph_msgs/Log.h>
#  define LOG_MSGTYPE  rosgraph_msgs::Log
#else
#  include <roslib/Log.h>
#  define LOG_MSGTYPE  roslib::Log
#endif

#include <ros/ros.h>
#include <yaml.h>
#ifdef USE_REGEX_CPP
// we do not use it atm because it does not work as epxect atm,
// cf. https://bugzilla.redhat.com/show_bug.cgi?id=718711
#  include <regex>
#else
#  include <regex.h>
#endif

class ROSoutFilter
{
 public:
  typedef enum {
    INCLUDE,
    EXCLUDE
  } FilterType;

  typedef enum {
    FILTER_FOR_NODENAME    =  1,
    FILTER_FOR_LEVELS      =  2,
    FILTER_FOR_REGEX       =  4,
  } FilterFlags;

  ROSoutFilter(ros::NodeHandle &nh, bool verbose = false);
  ~ROSoutFilter();

 private:  
  void logmsg_cb(const LOG_MSGTYPE::ConstPtr &msg);
  void read_config();
  uint8_t read_config_levels(const YAML::Node &n);

  void print_config();

  bool filtered(const LOG_MSGTYPE::ConstPtr &msg) const;

 private:
  ros::NodeHandle __nh;
  bool            __verbose;

  ros::Subscriber __sub_rosout;
  ros::Publisher  __pub_rosout;

  bool        __default_include;
  std::string __input_topic;
  std::string __output_topic;
  uint8_t      __levels;

  typedef struct {
    FilterType  type;
    FilterFlags flags;
    std::string nodename;
    uint8_t     levels;
    std::string regex_str;
#ifdef USE_REGEX_CPP
    std::regex  regex;
#else
    regex_t     regex;
#endif
  } FilterConfig;

  std::list<FilterConfig>  __filters;
};



#endif
