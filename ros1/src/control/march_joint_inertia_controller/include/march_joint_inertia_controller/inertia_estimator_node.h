// Created by hvaneven on 07-12-20. 15:54

#ifndf JOINT_INERTIA_CONTROLLER_H_JOINT_INERTIA_ESTIMATOR_H
#define JOINT_INERTIA_CONTROLLER_H_JOINT_INERTIA_ESTIMATOR_H

#endif //JOINT_INERTIA_CONTROLLER_H_JOINT_INERTIA_ESTIMATOR_H

#include <march_joint_inertia_controller/inertia_estimator.h>

//variables
double joint_inertia;
std_msgs::Double joint_inertia_msg;

//objects
InertiaEstimator inertia_estimator_object;

