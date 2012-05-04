#ifndef _KITEHELPER_H
#define _KITEHELPER_H

#include <KineoModel/KineoModel.h>

namespace KiteHelper {

bool initialize_kineo(bool verbose = false);
void append_kxml_to_tree (CkppModelTreeShPtr tree, const char *filename);

CkppDeviceComponentShPtr find_robot (CkppModelTreeShPtr tree);

void setup_collision_capsules_robot_robot (CkppDeviceComponentShPtr robot, bool verbose = false);
void setup_collision_capsules_robot_environment (CkppDeviceComponentShPtr robot, bool verbose = false);

void setup_collision_pairs_robot_environment (CkppModelTreeShPtr tree, CkppDeviceComponentShPtr robot, bool verbose = false);
void setup_collision_pairs_robot_robot (CkppDeviceComponentShPtr robot);

void setup_robot_steering_method (CkppDeviceComponentShPtr robot);
void setup_robot_penetration (CkppDeviceComponentShPtr robot, double penetration);

CkwsDiffusingRdmBuilderShPtr create_roadmap_builder (CkppDeviceComponentShPtr robot);

void print_robot_collision_pairs (CkppDeviceComponentShPtr robot);

CkwsConfig create_config (CkppDeviceComponentShPtr robot, const std::vector<double> &dof_values);
bool validate_config (CkppDeviceComponentShPtr robot, CkwsConfig config);

CkwsPathShPtr create_direct_path (CkppDeviceComponentShPtr robot, CkwsConfig start, CkwsConfig end);
CkwsPathShPtr create_path (CkppDeviceComponentShPtr robot, const std::vector<std::vector< double > > &configurations);

}

#endif
