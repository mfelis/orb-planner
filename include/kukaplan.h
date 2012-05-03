#ifndef _KUKA_PLAN_H
#define _KUKA_PLAN_H

#include <KineoModel/KineoModel.h>

bool initialize_kineo(bool verbose = false);
void append_kxml_to_tree (CkppModelTreeShPtr tree, const char *filename);

CkppDeviceComponentShPtr find_robot (CkppModelTreeShPtr tree);

void setup_collision_pairs_robot_environment (CkppModelTreeShPtr tree, CkppDeviceComponentShPtr robot);
void setup_collision_pairs_robot_robot (CkppDeviceComponentShPtr robot);
void setup_robot_steering_method (CkppDeviceComponentShPtr robot);
void setup_robot_penetration (CkppDeviceComponentShPtr robot, double penetration);
CkwsDiffusingRdmBuilderShPtr create_roadmap_builder (CkppDeviceComponentShPtr robot);

void print_robot_collision_pairs (CkppDeviceComponentShPtr robot);

CkwsConfig create_config (CkppDeviceComponentShPtr robot, const std::vector<double> &dof_values);
bool validate_config (CkppDeviceComponentShPtr robot, CkwsConfig config);

CkwsPathShPtr create_direct_path (CkppDeviceComponentShPtr robot, CkwsConfig start, CkwsConfig end);

namespace KukaPlan {

/** \brief Used to make results from the path planner available.
 */
struct PathQueryInformation {
	PathQueryInformation() : 
		collision(false)
	{}

	bool collision;
};

/** \brief Initializes the path planner and sets up the scene. */
bool kukaplan_initialize(const char* robot_file, const char* scene_file);

bool kukaplan_check_path (const std::vector<std::vector< double > > &configurations, PathQueryInformation *info = NULL);

/** \brief Plans a collision free using an initial guess. */
bool kukaplan_plan_path (std::vector<std::vector< double > > configurations_in, std::vector<std::vector< double> > path_out);

bool kukaplan_optimize_path (std::vector<std::vector< double > > configurations_in, std::vector<std::vector< double> > optimized_path_out);

}

#endif
