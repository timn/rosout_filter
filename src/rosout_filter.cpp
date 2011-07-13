
/***************************************************************************
 *  rosout_filter.cpp - rosout filter application
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

#include "rosout_filter.h"

#include <fstream>

inline ROSoutFilter::FilterFlags
operator|(ROSoutFilter::FilterFlags a, ROSoutFilter::FilterFlags b)
{
  return ROSoutFilter::FilterFlags(int(a) | int(b));
}
inline ROSoutFilter::FilterFlags &
operator|=(ROSoutFilter::FilterFlags &a, ROSoutFilter::FilterFlags b)
{
  a = a | b;
  return a;
}

inline ROSoutFilter::FilterFlags
operator&(ROSoutFilter::FilterFlags a, ROSoutFilter::FilterFlags b)
{
  return ROSoutFilter::FilterFlags(int(a) & int(b));
}

inline ROSoutFilter::FilterFlags operator&(ROSoutFilter::FilterFlags a, int b)
{
  return ROSoutFilter::FilterFlags(int(a) & b);
}

ROSoutFilter::ROSoutFilter(ros::NodeHandle &nh,
			   std::string &config_file, bool verbose)
  : __nh(nh), __config_file(config_file), __verbose(verbose)
{
  __sub_rosout = __nh.subscribe("/rosout_agg", 10,
				&ROSoutFilter::logmsg_cb, this);

  __pub_rosout = __nh.advertise<LOG_MSGTYPE>("/rosout_filtered", 10);

  read_config();
  if (__verbose)  print_config();
}

ROSoutFilter::~ROSoutFilter()
{
  __sub_rosout.shutdown();
  __pub_rosout.shutdown();
}


void
ROSoutFilter::logmsg_cb(const LOG_MSGTYPE::ConstPtr &msg)
{
  if (! filtered(msg)) {
    if (__verbose)  printf("INCLUDE: %s\n", msg->msg.c_str());
    __pub_rosout.publish(msg);
  } else {
    if (__verbose)  printf("EXCLUDE: %s\n", msg->msg.c_str());  }
}


bool
ROSoutFilter::filtered(const LOG_MSGTYPE::ConstPtr &msg) const
{
  std::list<FilterConfig>::const_iterator f;
  unsigned int i = 1;
  for (f = __filters.begin(); f != __filters.end(); ++f) {
    bool matched = true;
    if ((f->flags & FILTER_FOR_NODENAME) && (f->nodename != msg->name)) {
      matched = false;
    }
#ifdef USE_REGEX_CPP
    if ((f->flags & FILTER_FOR_REGEX) && ! regex_search(msg->msg, f->regex)) {
#else
      if ((f->flags & FILTER_FOR_REGEX) &&
	  regexec(&f->regex, msg->msg.c_str(), 0, NULL, 0) == REG_NOMATCH)
      {
#endif
      matched = false;
    }
    if ((f->flags & FILTER_FOR_LEVELS) && ! (f->levels & msg->level)) {
      matched = false;
    }

    if (matched) {
      if (__verbose)  printf("Rule %u matched\n", i);
      return (f->type == EXCLUDE);
    }
    ++i;
  }

  return ! __default_include;
}

uint8_t
ROSoutFilter::read_config_levels(const YAML::Node &n)
{
  uint8_t levels = 0;

  for (unsigned int i = 0; i < n.size(); ++i) {
    std::string level;
    n[i] >> level;
    if (level == "DEBUG") {
      levels |= LOG_MSGTYPE::DEBUG;
    } else if (level == "INFO") {
      levels |= LOG_MSGTYPE::INFO;
    } else if (level == "WARN") {
      levels |= LOG_MSGTYPE::WARN;
    } else if (level == "ERROR") {
      levels |= LOG_MSGTYPE::ERROR;
    } else if (level == "FATAL") {
      levels |= LOG_MSGTYPE::FATAL;
    } else {
      throw std::runtime_error(std::string("Unknown log level ") + level);
    }
  }

  return levels;
}

void
ROSoutFilter::read_config()
{
  std::ifstream fin(__config_file);
  YAML::Parser parser(fin);

  YAML::Node general_doc;
  if (! parser.GetNextDocument(general_doc)) {
    throw std::runtime_error("Failed to read document from file");
  }

  YAML::Node filter_doc;
  if (! parser.GetNextDocument(filter_doc)) {
    throw std::runtime_error("Failed to read configuration document from file");
  }

  // ***** Read general configuration

  __default_include = true;
  try {
    std::string default_type;
    general_doc["default"] >> default_type;
    if (default_type == "include") {
      __default_include = true;
    } else if (default_type == "exclude") {
      __default_include = false;
    } else {
      throw std::runtime_error("Unknown default action");
    }
  } catch (YAML::KeyNotFound &e) {} // ignored, use default

  __input_topic = "/rosout_agg";
  try {
    general_doc["input-topic"] >> __output_topic;
  } catch (YAML::KeyNotFound &e) {} // ignored, use default

  __output_topic = "/rosout_filtered";
  try {
    general_doc["output-topic"] >> __output_topic;
  } catch (YAML::KeyNotFound &e) {} // ignored, use default

  __levels = 0;
  try {
    __levels = read_config_levels(general_doc["levels"]);
  } catch (YAML::KeyNotFound &e) {
    // default to all levels
    __levels = LOG_MSGTYPE::DEBUG | LOG_MSGTYPE::INFO | LOG_MSGTYPE::WARN
      | LOG_MSGTYPE::ERROR | LOG_MSGTYPE::FATAL;
  }

  // ***** Read filter configuration
  for (unsigned int i = 0; i < filter_doc.size(); ++i) {
    const YAML::Node &c = filter_doc[i];

    FilterConfig fc;

    std::string regex;
    std::string type;

    c["type"] >> type;
    if (type == "include") {
      fc.type = INCLUDE;
    } else if (type == "exclude") {
      fc.type = EXCLUDE;
    } else {
      throw std::runtime_error("Unknown filter type " + type);
    }

    fc.flags = (FilterFlags)0;
    try {
      c["nodename"] >> fc.nodename;
      fc.flags |= FILTER_FOR_NODENAME;
    } catch (YAML::KeyNotFound &e) {} // ignored
    try {
      c["regex"] >> fc.regex_str;
#ifdef USE_REGEX_CPP
      fc.regex =  std::regex(fc.regex_str, std::regex_constants::extended);
#else
      if (regcomp(&fc.regex, fc.regex_str.c_str(), REG_EXTENDED) != 0) {
	throw std::runtime_error("Failed to compile regex");
      }
#endif
      fc.flags |= FILTER_FOR_REGEX;
    } catch (YAML::KeyNotFound &e) {} // ignored

    try {
      fc.levels = read_config_levels(c["levels"]);
      fc.flags |= FILTER_FOR_LEVELS;
    } catch (YAML::KeyNotFound &e) {} // ignored

    __filters.push_back(fc);
  }
}


void
ROSoutFilter::print_config()
{
  printf("Input topic:  %s\n"
	 "Output topic: %s\n"
	 "Log levels:   %u\n"
	 "Filter config:\n",
	 __input_topic.c_str(), __output_topic.c_str(), __levels);

  std::list<FilterConfig>::iterator f;
  unsigned int i = 1;
  for (f = __filters.begin(); f != __filters.end(); ++f) {
    printf("  Number:    %u\n", i++);
    printf("  Type:      %s\n", (f->type == INCLUDE) ? "INCLUDE" : "EXCLUDE");
    if (f->flags & FILTER_FOR_NODENAME) {
      printf("  Nodename:  %s\n", f->nodename.c_str());
    }
    if (f->flags & FILTER_FOR_REGEX) {
      printf("  RegEx:     %s\n", f->regex_str.c_str());
    }
    if (f->flags & FILTER_FOR_LEVELS) {
      printf("  Levels:    %u\n", f->levels);
    }
    printf("\n");
  }
}
