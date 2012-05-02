#ifndef _KUKA_PLAN_H
#define _KUKA_PLAN_H

#include <KineoModel/KineoModel.h>

bool initialize_kineo(bool verbose = false);
void append_kxml_to_tree (CkppModelTreeShPtr tree, const char *filename);

CkppDeviceComponentShPtr find_robot (CkppModelTreeShPtr tree);

void setup_collision_pairs_robot_environment (CkppModelTreeShPtr tree, CkppDeviceComponentShPtr robot);
void setup_collision_pairs_robot_robot (CkppModelTreeShPtr tree, CkppDeviceComponentShPtr robot);

void print_robot_collision_pairs (CkppDeviceComponentShPtr robot);

#endif
