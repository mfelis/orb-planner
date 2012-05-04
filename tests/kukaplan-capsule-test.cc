// Copyright (C) 2011, 2012 by Antonio El Khoury.
//
// This file is part of the orb-planner.
//
// orb-planner is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// orb-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with orb-planner.  If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE plan

#include <iomanip>

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "kukaplan.h"

using boost::test_tools::output_test_stream;

// Define path to the files that should be loaded
#define ORB_PLANNER_DIR "/home/mfelis/local/src/orb-planner"

#define ORB_PLANNER_ROBOT_FILE ORB_PLANNER_DIR"/data/KUKA_sixx850.kxml"
#define ORB_PLANNER_OBSTACLE_FILE ORB_PLANNER_DIR"/data/test_planning_only_obstacles.kxml"

using namespace KukaPlan;
using namespace std;

BOOST_AUTO_TEST_CASE (check_direct_path)
{
	kukaplan_initialize_capsules(ORB_PLANNER_ROBOT_FILE, ORB_PLANNER_OBSTACLE_FILE);

	exit (1);
	// Set initial pose of the robot.
	std::vector<double> startDofValues (6, 0.);
	startDofValues[0] = 0.;
	startDofValues[1] = - 90. * M_PI / 180.;
	startDofValues[2] = 0.;
	startDofValues[3] = 0.;
	startDofValues[4] = 0.;
	startDofValues[5] = 0.;

// Set target pose of the robot.
	std::vector<double> goalDofValues (6);

	//	goalDofValues[0] =   10. * M_PI / 180.; 
	//  goalDofValues[1] =   0. * M_PI / 180.;  
	//  goalDofValues[2] =   0. * M_PI / 180.;
	//  goalDofValues[3] =   0. * M_PI / 180.; 
	//  goalDofValues[4] =   0. * M_PI / 180.; 
	//  goalDofValues[5] =   0. * M_PI / 180.; 

	goalDofValues[0] =   0. * M_PI / 180.; 
	goalDofValues[1] =  -0. * M_PI / 180.;
	goalDofValues[2] =   0. * M_PI / 180.;
	goalDofValues[3] =   0. * M_PI / 180.; 
	goalDofValues[4] =   0. * M_PI / 180.; 
	goalDofValues[5] =   0. * M_PI / 180.; 

	std::vector<std::vector< double > > configurations;
	configurations.push_back(startDofValues);
	configurations.push_back(goalDofValues);

	if (!kukaplan_check_path (configurations)) {
		cout << "Collision!" << endl;
	}
}

