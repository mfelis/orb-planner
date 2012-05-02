#ifndef _KUKA_PLAN_H
#define _KUKA_PLAN_H

#include <KineoModel/KineoModel.h>

bool initialize_kineo(bool verbose = false);
void append_kxml_to_tree (CkppModelTreeShPtr tree, const char *filename);

#endif
