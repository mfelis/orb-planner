#ifndef _KUKA_PLAN_H
#define _KUKA_PLAN_H

#include <KineoModel/KineoModel.h>

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
bool kukaplan_plan_path (const std::vector<std::vector< double > > &configurations_in, std::vector<std::vector< double> > &path_out);

bool kukaplan_optimize_path (const std::vector<std::vector< double > > &configurations_in, std::vector<std::vector< double> > &optimized_path_out);

}

#endif
