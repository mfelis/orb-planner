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

#include <iostream>
#include <iomanip>

#include <KineoUtility/kitParameterMap.h>
#include <KineoModel/KineoModel.h>
#include <KineoController/KineoController.h>
#include <KineoModuleManager/KineoModuleManager.h>
#include <kprParserXML/KineoParserXML.h>
#include <KineoWorks2/kwsDiffusingRdmBuilder.h>
#include <kwsKcd2/kwsKCDBody.h>

#include "kitehelper.h"
#include "kukaplan.h"

using namespace std;
using namespace KiteHelper;
using namespace hpp::geometry::component;

namespace KukaPlan {

CkppModelTreeShPtr modelTree;
CkppDeviceComponentShPtr robot;
CkwsDiffusingRdmBuilderShPtr roadmapBuilder;
std::map<std::string, hpp::geometry::component::CapsuleShPtr> jointNameCapsuleMap;

bool kukaplan_initialize(const char* robot_file, const char* scene_file) {
	if (!initialize_kineo()) {
		std::cerr << "Failed to validate Kineo license." << std::endl;
		exit(1);
		return false;
	}

	CkppModelTreeShPtr modelTree = CkppModelTree::create ();

	append_kxml_to_tree (modelTree, robot_file);
	append_kxml_to_tree (modelTree, scene_file);

	robot = find_robot (modelTree);

	setup_collision_pairs_robot_environment (modelTree, robot);
	setup_collision_pairs_robot_robot (robot);

	setup_robot_steering_method (robot);
	setup_robot_penetration (robot, 1.0e-3);

	roadmapBuilder = create_roadmap_builder (robot);

	return true;
}

void initialize_capsule_map () {
	// Base
	CapsuleShPtr Base_capsule = Capsule::create ("Base_capsule",
			CkitPoint3 (-0.249, 0.0, 0.543),
			CkitPoint3 (-0.113, 0., 0.0959),
			0.1606);
	Base_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["Base"] = Base_capsule;

	// B1
	CapsuleShPtr B1_capsule = Capsule::create ("B1_capsule",
			CkitPoint3 (0.0032, 0.0000, 0.2437),
			CkitPoint3 (0.0750, -0.0097, 0.3350),
			0.1192);
	B1_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["B1"] = B1_capsule;

	// B2
	CapsuleShPtr B2_capsule = Capsule::create ("B2_capsule",
			CkitPoint3 (0.0750, 0.1041, 0.3350),
			CkitPoint3 (0.4357, 0.1097, 0.3288),
			0.0905);
	B2_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["B2"] = B2_capsule;

	// B3
	CapsuleShPtr B3_capsule = Capsule::create ("B3_capsule",
			CkitPoint3 (0.4405, -0.0048, 0.3410),
			CkitPoint3 (0.5812, -0.0040, 0.4250),
			0.1072);
	B3_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["B3"] = B3_capsule;

	// B4
	CapsuleShPtr B4_capsule = Capsule::create ("B4_capsule",
			CkitPoint3 (0.6893, 0.0047, 0.4279),
			CkitPoint3 (0.8176, -0.0040, 0.4311),
			0.0874);
	B4_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["B4"] = B4_capsule;

	// B5
	CapsuleShPtr B5_capsule = Capsule::create ("B5_capsule",
			CkitPoint3 (0.8450, 0.0000, 0.4250),
			CkitPoint3 (0.8597, 0.0000, 0.4250),
			0.0495);
	B5_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["B5"] = B5_capsule;

	// B6
	CapsuleShPtr B6_capsule = Capsule::create ("B6_capsule",
			CkitPoint3 (0.9148, -0.0010, 0.4249),
			CkitPoint3 (0.9148, 0.0000, 0.4248),
			0.0372);
	B6_capsule->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	jointNameCapsuleMap["B6"] = B6_capsule;
}

bool kukaplan_initialize_capsules(const char* robot_file, const char* scene_file) {
	if (!initialize_kineo()) {
		std::cerr << "Failed to validate Kineo license." << std::endl;
		exit(1);
		return false;
	}

	CkppModelTreeShPtr modelTree = CkppModelTree::create ();

	append_kxml_to_tree (modelTree, robot_file);
	append_kxml_to_tree (modelTree, scene_file);

	robot = find_robot (modelTree);

	initialize_capsule_map();

	setup_collision_capsules_robot_robot (robot, jointNameCapsuleMap, true);
//	setup_collision_pairs_robot_environment (modelTree, robot);

	setup_robot_steering_method (robot);
	setup_robot_penetration (robot, 1.0e-3);

	roadmapBuilder = create_roadmap_builder (robot);

	return true;
}


bool kukaplan_check_path (const std::vector<std::vector< double > > &configurations, PathQueryInformation *info) {
	assert (robot);

	CkwsPathShPtr path = create_path (robot, configurations);

	if (robot->pathValidators()->validate(*path)) {
		return true;
	}

	if (info) {
		info->collision = true;
	}
	
	return false;
}

bool kukaplan_plan_path (const std::vector<std::vector< double > > &configurations_in, std::vector<std::vector< double> > &path_out) {
	assert (robot);

	bool found_path = false;

	CkwsPathShPtr path = create_path (robot, configurations_in);
	CkwsPathShPtr solution_path = CkwsPath::createCopy(path);

	if (KD_OK == roadmapBuilder ->solveProblem (*(path->configAtStart()), *(path->configAtEnd()), solution_path))
		found_path = true;
	else {
		std::cout << "ERROR" << std::endl;
	}

	path_out.clear();

	for (unsigned int i = 0; i < solution_path->countConfigurations(); ++i) {
		CkwsConfig config (robot);
		solution_path->getConfiguration (i, config);
		std::vector<double> dof_config(robot->countDofs());

		for (unsigned int j = 0; j < config.size(); ++j) {
			dof_config[j] = config.dofValue(j);
		}
		path_out.push_back(dof_config);
	}

	return found_path;
}

bool kukaplan_optimize_path (const std::vector<std::vector< double > > &configurations_in, std::vector<std::vector< double> > &optimized_path_out) {
	assert (robot);
	bool optimize_success = false;

	CkwsPathShPtr path = create_path (robot, configurations_in);
	if (robot->pathValidators()->validate(*path) == false) {
		std::cout << "Warning: cannot optimize invalid path!" << endl;
		return false;
	}

	CkwsPathShPtr optimized_path = CkwsPath::createCopy(path);

	CkwsRandomOptimizerShPtr optimizer = CkwsRandomOptimizer::create ();

	if (KD_OK == optimizer->optimizePath(optimized_path)) {
		optimize_success = true;
	}

	optimized_path_out.clear();
	for (unsigned int i = 0; i < optimized_path->countConfigurations(); ++i) {
		CkwsConfig config (robot);
		optimized_path->getConfiguration (i, config);
		std::vector<double> dof_config(robot->countDofs());

		for (unsigned int j = 0; j < config.size(); ++j) {
			dof_config[j] = config.dofValue(j);
		}
		optimized_path_out.push_back(dof_config);
	}

	return optimize_success;
}


}
