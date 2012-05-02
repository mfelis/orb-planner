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

#include "parser.cc"
#include "kukaplan.h"

// Define where the device loading libraries are. Make sure you load
// the correct ones (depending on whether you're using the release or
// debug libraries).
#define KINEO_INSTALL_DIR "/home/mfelis/local/robotpkg/kineo-2.06"

#define KINEODEVICEPARSING_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviceParsingd.so"
#define KINEODEVICEBASE_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviceBased.so"
#define KINEODEVICE_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviced.so"

// Define path to the files that should be loaded
#define ORB_PLANNER_DIR "/home/mfelis/local/src/orb-planner"

#define ORB_PLANNER_ROBOT_FILE ORB_PLANNER_DIR"/data/KUKA_sixx850.kxml"
#define ORB_PLANNER_OBSTACLE_FILE ORB_PLANNER_DIR"/data/test_planning_only_obstacles.kxml"

using namespace std;

bool initialize_kineo(bool verbose) {
	  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return false;
    }

  // ----------------------------------------------------------------

  // Initialize module manager to allow parsing device.
  CkppModuleManagerShPtr moduleManager = CkppModuleManager::create ();
  moduleManager->addModuleFile (KINEODEVICEPARSING_SO);
  moduleManager->addModuleFile (KINEODEVICEBASE_SO);
  moduleManager->addModuleFile (KINEODEVICE_SO);

  CkprParserManager::defaultManager ()->moduleManager (moduleManager);

  moduleManager->initializeModules ();

  if (moduleManager->countModules () == 0) {
    std::cout << "No module loaded. Are you sure you the modules paths are correct?" << std::endl;
	} else {
		if (verbose) {
			for (unsigned int i=0; i < moduleManager->countModules (); i++)
				std::cout << "Module " << i << ": " << moduleManager->module (i)->name () << std::endl;
		}
	}

	return true;
}

void append_kxml_to_tree (CkppModelTreeShPtr tree, const char *filename) {
  // Create a parser instance.
  CkprParserManagerShPtr parser = CkprParserManager::defaultManager ();

  CkppDocumentShPtr document =
    CkppDocument::create (CkprParserManager::defaultManager()
			  ->moduleManager ());
  CkppComponentFactoryRegistryShPtr registry
    = document->componentFactoryRegistry ();

	parseFile (filename, parser, registry, tree);
}

CkppDeviceComponentShPtr find_robot (CkppModelTreeShPtr tree) {
  // Assuming there is one robot in the model tree, retrieve it.
  assert (tree->deviceNode ()->countChildComponents () == 1
	  && "Wrong number of devices in model tree, expected 1.");

  CkppDeviceComponentShPtr robot
    = KIT_DYNAMIC_PTR_CAST (CkppDeviceComponent,
			    tree->deviceNode ()->childComponent (0));
  assert (!!robot && "Null pointer to robot.");

	return robot;
}

void setup_collision_pairs_robot_environment (CkppModelTreeShPtr tree, CkppDeviceComponentShPtr robot) {
  // A body of the robot is said to be in collision if any of the
  // outer objects collide with any of the inner objects. Usually you
  // have only one inner objet, and multiple outer objects, such as
  // objects from the environment, or other bodies of the same robot.

  // Store all geometry component references of device to avoid adding
  // them as outer objects. This will allows us later on to add only
  // obstacles. Self-collision pairs can be then defined separately.
	std::vector<CkppSolidComponentRefShPtr> solidComponentRefVector;
	robot->getSolidComponentRefVector (solidComponentRefVector);

	// Add all activated obstacles in model tree as outer objects of
	// each joint body.
	for (unsigned i = 0;
			i < tree->geometryNode ()->countChildComponents ();
			++i)
	{
		CkppSolidComponentShPtr solidComponent
			= KIT_DYNAMIC_PTR_CAST (CkppSolidComponent,
					tree->geometryNode ()->childComponent (i));
		assert (!!solidComponent && "Null pointer to solidComponent.");

		unsigned j = 0;
		bool isSolidComponentInRobot = false;
		while (j < solidComponentRefVector.size ()
				&& isSolidComponentInRobot == false)
		{
			CkppSolidComponentShPtr robotSolidComponent
				= solidComponentRefVector[j]->referencedSolidComponent ();
			assert (!!robotSolidComponent
					&& "Null pointer to robot solid component.");
			if (robotSolidComponent == solidComponent)
			{
				std::cout << "solid component '" << robotSolidComponent->name()
					<< "' matches body in joint " << j << std::endl;
				isSolidComponentInRobot = true;
			}
			++j;
		}

		CkwsDevice::TJointVector jointVector;
		robot->getJointVector (jointVector);

		if (!isSolidComponentInRobot && solidComponent->isActivated ())
			for (unsigned j = 0; j < jointVector.size (); ++j)
			{
				std::cout << "Adding solid (name = '" << solidComponent->name () << "') to joint index: " << j << std::endl;

				CkcdObjectShPtr object
					= KIT_DYNAMIC_PTR_CAST (CkcdObject,
							solidComponent);
				assert (!!object && "Null pointer to object.");

				assert (!!jointVector[j]->attachedBody ()
						&& "Null pointer to attached body.");
				CkwsKCDBodyShPtr body
					= KIT_DYNAMIC_PTR_CAST (CkwsKCDBody,
							jointVector[j]->attachedBody ());
				assert (!!body && "Null pointer to body.");
				std::vector<CkcdObjectShPtr> outerObjects = body->outerObjects ();
				outerObjects.push_back (object);
				body->outerObjects (outerObjects);
			}
	}
}

void setup_collision_pairs_robot_robot (CkppModelTreeShPtr tree, CkppDeviceComponentShPtr robot) {
	std::vector<CkppSolidComponentRefShPtr> solidComponentRefVector;
	robot->getSolidComponentRefVector (solidComponentRefVector);

	// To avoid self-collision, add robot bodies as outer bodies. Here
	// we add all collision pairs. This is optional, and it may be nice
	// to find automatically which collision pairs should be taken into
	// consideration.
	for (unsigned i = 0; i < solidComponentRefVector.size (); ++i)
	{
		CkppSolidComponentShPtr robotSolidComponent1
			= solidComponentRefVector[i]->referencedSolidComponent ();
		assert (!!robotSolidComponent1
				&& "Null pointer to robot solid component 1.");
		if (robotSolidComponent1->isActivated ())
		{
			CkwsDevice::TJointVector jointVector;
			robot->getJointVector (jointVector);
			CkwsKCDBodyShPtr body
				= KIT_DYNAMIC_PTR_CAST (CkwsKCDBody,
						jointVector[i]->attachedBody ());
			std::vector<CkcdObjectShPtr> outerObjects = body->outerObjects ();

			for (unsigned j = i + 2; j < solidComponentRefVector.size (); ++j)
			{
				CkppSolidComponentShPtr robotSolidComponent
					= solidComponentRefVector[j]->referencedSolidComponent ();
				assert (!!robotSolidComponent
						&& "Null pointer to robot solid component.");
				if (robotSolidComponent->isActivated ())
				{
					CkcdObjectShPtr robotObject
						= KIT_DYNAMIC_PTR_CAST (CkcdObject,
								robotSolidComponent);
					assert (!!robotObject
							&& "Null pointer to robot object.");
					outerObjects.push_back (robotObject);
				}
			}

			body->outerObjects (outerObjects);
		}
	}
}

void print_robot_collision_pairs (CkppDeviceComponentShPtr robot) {
  // Print inner and outer objects for each joint of the robot.
  CkwsDevice::TJointVector jointVector;
  robot->getJointVector (jointVector);
  for (unsigned i = 0; i < jointVector.size (); ++i)
    {
      std::cout << "Joint " << i << std::endl;
      CkwsKCDBodyShPtr body
	= KIT_DYNAMIC_PTR_CAST (CkwsKCDBody,
				jointVector[i]->attachedBody ());
      assert (!!body && "Null pointer to body.");

      std::vector<CkcdObjectShPtr> innerObjects = body->innerObjects ();
      std::vector<CkcdObjectShPtr> outerObjects = body->outerObjects ();

      std::cout << "Number of inner objects: "
		<< innerObjects.size () << std::endl;
      std::cout << "Number of outer objects: "
		<< outerObjects.size () << std::endl;
    }
}
