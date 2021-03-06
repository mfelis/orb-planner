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

#include <KineoUtility/kitParameterMap.h>
#include <KineoModel/KineoModel.h>
#include <KineoController/KineoController.h>
#include <KineoModuleManager/KineoModuleManager.h>
#include <kprParserXML/KineoParserXML.h>
#include <KineoWorks2/kwsDiffusingRdmBuilder.h>
#include <kwsKcd2/kwsKCDBody.h>

#include "parser.cc"
#include "kitehelper.h"

using boost::test_tools::output_test_stream;
using namespace KiteHelper;

// Define path to the files that should be loaded
#define ORB_PLANNER_DIR "/home/mfelis/local/src/orb-planner"

#define ORB_PLANNER_ROBOT_FILE ORB_PLANNER_DIR"/data/KUKA_sixx850.kxml"
#define ORB_PLANNER_OBSTACLE_FILE ORB_PLANNER_DIR"/data/test_planning_only_obstacles.kxml"

// This is the main program.
BOOST_AUTO_TEST_CASE (plan)
{
	if (!initialize_kineo (true)) {
		std::cout << "Failed to validate Kineo license." << std::endl;
		return;
	}

	CkppModelTreeShPtr modelTree = CkppModelTree::create ();

	append_kxml_to_tree (modelTree, ORB_PLANNER_OBSTACLE_FILE);
	append_kxml_to_tree (modelTree, ORB_PLANNER_ROBOT_FILE);

	//  printComponent (modelTree);

	CkppDeviceComponentShPtr robot = find_robot (modelTree);

	setup_collision_pairs_robot_environment (modelTree, robot);
	setup_collision_pairs_robot_robot (robot);

	setup_robot_steering_method (robot);
	setup_robot_penetration (robot, 1.0e-3);

	CkwsDiffusingRdmBuilderShPtr roadmapBuilder = create_roadmap_builder (robot);

	//	print_robot_collision_pairs (robot);

	// Set initial pose of the robot.
	std::vector<double> startDofValues (6, 0.);
	startDofValues[0] = 0.;
	startDofValues[1] = - 90. * M_PI / 180.;
	startDofValues[2] = 0.;
	startDofValues[3] = 0.;
	startDofValues[4] = 0.;
	startDofValues[5] = 0.;

	CkwsConfig startConfig = create_config (robot, startDofValues);

	std::cout << "Validation of start config: ";
	if (validate_config (robot, startConfig)) {
		std::cout << "valid!" << std::endl;
	} else {
		std::cout << "invalid!" << std::endl;
	}

	assert (startConfig.isValid () == true
			&& "Start configuration is not collision free, should be.");

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

	CkwsConfig goalConfig = create_config (robot, goalDofValues);


	// Validation of end config 
	std::cout << "Validation of goal config : ";

	if (validate_config (robot, goalConfig)) {
		std::cout << "valid!" << std::endl;
	} else {
		std::cout << "invalid!" << std::endl;
	}

	assert (goalConfig.isValid () == true
			&& "Goal configuration is not collision free, should be.");

	// compute the intermediate state of the start and goal state and check
	// for collision (we want to have one!)

	// Set target pose of the robot.
	CkwsConfig midConfig (robot);
	std::vector<double> midDofValues (6);

	// interesting path for "test_planning_only_obstacles
	midDofValues[0] = startDofValues[0] + 0.5 * (goalDofValues[0] - startDofValues[0]); 
	midDofValues[1] = startDofValues[1] + 0.5 * (goalDofValues[1] - startDofValues[1]);  
	midDofValues[2] = startDofValues[2] + 0.5 * (goalDofValues[2] - startDofValues[2]);
	midDofValues[3] = startDofValues[3] + 0.5 * (goalDofValues[3] - startDofValues[3]); 
	midDofValues[4] = startDofValues[4] + 0.5 * (goalDofValues[4] - startDofValues[4]); 
	midDofValues[5] = startDofValues[5] + 0.5 * (goalDofValues[5] - startDofValues[5]); 

	// std::cout << "midDofValues:\t";
	// for (int i = 0; i < 6; i++) {
	// 	std::cout << midDofValues[i] << "\t";
	// }
	// std::cout << std::endl;

	midConfig.setDofValues (midDofValues);

	// Validation of end config 
	std::cout << "Validation of mid config  : ";
	robot->configValidators()->validate(midConfig);
	if (midConfig.isValid()) {
		std::cout << "valid!" << std::endl;
	} else {
		std::cout << "invalid!" << std::endl;
	}

	assert (midConfig.isValid () == false
			&& "Mid configuration is not in collision, but should be.");


	// ----------------------------------------------------------------
	// Create initial path from start and goal configurations.
	CkwsPathShPtr initPath = create_direct_path (robot, startConfig, goalConfig);

	assert (!!initPath && "Null pointer to initial path.");
	if (initPath->countDirectPaths() != 1) {
		std::cerr << "Wrong number of direct paths in initial path, expected 1 but was " << initPath->countDirectPaths() << std::endl;
		assert (initPath->countDirectPaths () == 1
				&& "Wrong number of direct paths in initial path, expected 1.");
	}

	if (robot->pathValidators ()->validate (*initPath)) {
		std::cout << "Init path validation successful!" << std::endl;
	} else {
		std::cout << "Init path validation failed!" << std::endl;
	}

	// assert (initPath->validateWithPenetration (0.01) == false
	// 	  && "Init path is not collliding, no point in planning.");

	// Plan collision-free path from start and goal configurations.
	std::cout << "Solving... " << std::endl;
	CkwsPathShPtr solutionPath = CkwsPath::createCopy (initPath);
	if (KD_OK == roadmapBuilder ->solveProblem (startConfig, goalConfig, solutionPath))
		std::cout << "OK" << std::endl;
	else
		std::cout << "ERROR" << std::endl;

	// Print solution path.
	std::cout << "solution path: " << std::endl;
	for (unsigned i = 0; i < solutionPath->countConfigurations (); ++i)
	{
		CkwsConfig config (robot);
		solutionPath->getConfiguration (i, config);
		std::cout << std::setw (5) << "q(" << i << "):";
		for (unsigned j = 0; j < config.size (); ++j)
			std::cout << std::setw (13) << config.dofValue (j);
		std::cout << std::endl;
	}

	// Check for collisions on solution path. Penetration is a kind of
	// tolerance when checking for collisions. The lowest it is, the
	// safer the result, but the slower the validation.
	assert (robot->pathValidators ()->validate (*solutionPath) == true
			&& "Solution path is collliding, there is a problem.");

	// ----------------------------------------------------------------

	// Optimize path using Kineo random optimizer to shorten path
	// length. It is a good idea to use this optimizer on the solution
	// path to have a nicer initial guess for MUSCOD (even if the
	// objective function does not take path length into consideration).
	CkwsPathShPtr optimizedPath = CkwsPath::createCopy (solutionPath);
	CkwsRandomOptimizerShPtr optimizer = CkwsRandomOptimizer::create ();
	std::cout << "Optimizing..." << std::endl;
	if (KD_OK == optimizer->optimizePath (optimizedPath))
		std::cout << "OK" << std::endl;
	else
		std::cout << "ERROR" << std::endl;

	// Print solution path.
	std::cout << "optimized path: " << std::endl;
	for (unsigned i = 0; i < optimizedPath->countConfigurations (); ++i)
	{
		CkwsConfig config (robot);
		optimizedPath->getConfiguration (i, config);
		std::cout << std::setw (5) << "q(" << i << "):";
		for (unsigned j = 0; j < config.size (); ++j)
			std::cout << std::setw (13) << config.dofValue (j);
		std::cout << std::endl;
	}

	assert (robot->pathValidators ()->validate (*optimizedPath) == true
			&& "Solution path is collliding, there is a problem.");

	// ----------------------------------------------------------------

	// Read path from MUSCOD Data and create kineo path.

	// You should in this case parse MUSCOD data, create
	// configurations, create direct paths with each pair of
	// configurations, and append each direct path to a path starting
	// with an empty one.
}
