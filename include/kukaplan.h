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

/** \brief Initializes the path planner and sets up the scene but uses
 * capsules instead of the actual robot geometry. */
bool kukaplan_initialize_capsules(const char* robot_file, const char* scene_file);

bool kukaplan_validate_configuration (const std::vector<double> &configuration);

/** \brief Checks whether all configurations are collision free.
 *
 * If the parameter index_out is specified and a configuration was invalid
 * then the variable will be set to the index of the configuration that is
 * invalid.
 *
 * \return true if all configurations are collision free, false otherwise
 * */
bool kukaplan_validate_configurations (const std::vector<std::vector< double > > &configurations, unsigned int *index_out = NULL);

bool kukaplan_check_path (const std::vector<std::vector< double > > &configurations, PathQueryInformation *info = NULL);

/** \brief Plans a collision free using an initial guess. */
bool kukaplan_plan_path (const std::vector<std::vector< double > > &configurations_in, std::vector<std::vector< double> > &path_out);

bool kukaplan_optimize_path (const std::vector<std::vector< double > > &configurations_in, std::vector<std::vector< double> > &optimized_path_out);

}

#endif
