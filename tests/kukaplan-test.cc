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
#define ORB_PLANNER_EMPTY_SCENE_FILE ORB_PLANNER_DIR"/data/empty_scene.kxml"

using namespace KukaPlan;
using namespace std;

BOOST_AUTO_TEST_CASE (validate_configuration_zero)
{
	cout << "---- Entering " << __func__ << endl;
	kukaplan_initialize(ORB_PLANNER_ROBOT_FILE, ORB_PLANNER_EMPTY_SCENE_FILE);

	// Set initial pose of the robot.
	std::vector<double> startDofValues (6, 0.);
	startDofValues[0] = 0.;
	startDofValues[1] = 0.;
	startDofValues[2] = 0.;
	startDofValues[3] = 0.;
	startDofValues[4] = 0.;
	startDofValues[5] = 0.;

	BOOST_CHECK (kukaplan_validate_configuration (startDofValues));	

	cout << "---- Exiting " << __func__ << endl;
}

/*
BOOST_AUTO_TEST_CASE (check_direct_path_not_colliding)
{
	cout << "---- Entering " << __func__ << endl;
	kukaplan_initialize(ORB_PLANNER_ROBOT_FILE, ORB_PLANNER_EMPTY_SCENE_FILE);

	// Set initial pose of the robot.
	std::vector<double> startDofValues (6, 0.);
	startDofValues[0] = 0.;
	startDofValues[1] = 0.;
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
	goalDofValues[1] =   0. * M_PI / 180.;
	goalDofValues[2] =   0. * M_PI / 180.;
	goalDofValues[3] =   0. * M_PI / 180.; 
	goalDofValues[4] =   0. * M_PI / 180.; 
	goalDofValues[5] =   0. * M_PI / 180.; 

	std::vector<std::vector< double > > configurations;
	configurations.push_back(startDofValues);
	configurations.push_back(goalDofValues);

	unsigned int validate_result = 0;
	if (!validate_configurations (configurations, &validate_result)) {
		cerr << "Configuration " << validate_result << " is invalid!" << endl;
		abort();
	}

	if (kukaplan_check_path (configurations) == false) {
		cout << "Collision!" << endl;
	} else {
		abort();
	}
	cout << "---- Exiting " << __func__ << endl;
}

BOOST_AUTO_TEST_CASE (check_direct_path)
{
	cout << "---- Entering " << __func__ << endl;
	kukaplan_initialize(ORB_PLANNER_ROBOT_FILE, ORB_PLANNER_OBSTACLE_FILE);

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
	goalDofValues[1] =   0. * M_PI / 180.;
	goalDofValues[2] =   0. * M_PI / 180.;
	goalDofValues[3] =   0. * M_PI / 180.; 
	goalDofValues[4] =   0. * M_PI / 180.; 
	goalDofValues[5] =   0. * M_PI / 180.; 

	std::vector<std::vector< double > > configurations;
	configurations.push_back(startDofValues);
	configurations.push_back(goalDofValues);

	if (!kukaplan_check_path (configurations)) {
		cout << "Collision!" << endl;
	} else {
		abort();
	}
	cout << "---- Exiting " << __func__ << endl;
}

BOOST_AUTO_TEST_CASE (plan_triple_path)
{
	kukaplan_initialize(ORB_PLANNER_ROBOT_FILE, ORB_PLANNER_OBSTACLE_FILE);

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
	goalDofValues[1] =  -92. * M_PI / 180.;
	goalDofValues[2] =   0. * M_PI / 180.;
	goalDofValues[3] =   0. * M_PI / 180.; 
	goalDofValues[4] =   0. * M_PI / 180.; 
	goalDofValues[5] =   0. * M_PI / 180.; 

	std::vector<double> midDofValues (6);

	// interesting path for "test_planning_only_obstacles
	midDofValues[0] = startDofValues[0] + 0.5 * (goalDofValues[0] - startDofValues[0]); 
	midDofValues[1] = startDofValues[1] + 0.5 * (goalDofValues[1] - startDofValues[1]);  
	midDofValues[2] = startDofValues[2] + 0.5 * (goalDofValues[2] - startDofValues[2]);
	midDofValues[3] = startDofValues[3] + 0.5 * (goalDofValues[3] - startDofValues[3]); 
	midDofValues[4] = startDofValues[4] + 0.5 * (goalDofValues[4] - startDofValues[4]); 
	midDofValues[5] = startDofValues[5] + 0.5 * (goalDofValues[5] - startDofValues[5]); 

	std::vector<std::vector< double > > configurations;
	configurations.push_back(startDofValues);
	configurations.push_back(midDofValues);
	configurations.push_back(goalDofValues);

	if (!kukaplan_check_path (configurations)) {
		cout << "Collision!" << endl;
	}
}

BOOST_AUTO_TEST_CASE (plan_direct_path)
{
	kukaplan_initialize(ORB_PLANNER_ROBOT_FILE, ORB_PLANNER_OBSTACLE_FILE);

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

	std::vector<std::vector< double > > solution;

	cout << "Planning .... " << endl;
	if (kukaplan_plan_path (configurations, solution)) {
		cout << "Found a path!" << endl;
	} else {
		cout << "Did not find a path!" << endl;
	}

	cout << "Solution: " << endl;
	for (unsigned int i = 0; i < solution.size(); i++) {
		cout << setw(3) << i << ": ";

		for (unsigned int j = 0; j < solution[i].size(); j++) {
			cout << setw(13) << solution[i][j] << " ";
		}

		cout << endl;
	}

	std::vector<std::vector< double > > optimized_solution;

	cout << "Optimizing ..." << endl;
	if (kukaplan_optimize_path (solution, optimized_solution)) {
		cout << "Optimization success!" << endl;
	} else {
		cout << "Optimization failed!" << endl;
	}

	cout << "Optimized: " << endl;
	for (unsigned int i = 0; i < optimized_solution.size(); i++) {
		cout << setw(3) << i << ": ";

		for (unsigned int j = 0; j < optimized_solution[i].size(); j++) {
			cout << setw(13) << optimized_solution[i][j] << " ";
		}

		cout << endl;
	}
}
*/
