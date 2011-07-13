
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

#include <unistd.h>


void
usage(const char *progname, const char *additional = 0)
{
  printf("Usage: %s [config_file]\n\n%s", progname,
	 additional ? additional : "");
}

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "rosout_filter");
  ros::NodeHandle n;
  ros::NodeHandle priv_n("~");

  std::string config_file;

  bool show_help = false;
  bool verbose = false;
  bool unknown_arg = false;
  int c;
  while ((c = getopt(argc, argv, "dh")) != -1) {
    if (c == '?') {
      unknown_arg = true;
    } else if (c == 'd') {
      verbose = true;
    } else if (c == 'h') {
      show_help = true;
    }
  }

  if (show_help) {
    usage(argv[0]);
    exit(0);
  }
  if (unknown_arg) {
    usage(argv[0], "Unknown argument\n");
    exit(-1);
  }

  int ind = optind;
  if (ind < argc) {
    config_file = argv[ind];
  } else if (priv_n.hasParam("config_file")) {
    priv_n.getParam("config_file", config_file);
  }
  if (priv_n.hasParam("verbose")) {
    priv_n.getParam("verbose", verbose);
  }

  if (config_file == "") {
    usage(argv[0],
	  "Cannot determine configuration file. Either pass as command line\n"
	  "parameter or set config_file parameter.\n");
    exit(-2);
  }

  ROSoutFilter rosout_filter(n, config_file, verbose);

  ros::spin();
}
