#include "model_predictive_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace model_predictive_controller {

    bool ModelPredictiveController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        // get joint name from the parameter server
        std::string joint;
        if (!n.getParam("joint", joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }

        // get the joint object to use in the realtime loop
        joint_ = hw->getHandle(joint);  // throws on failure
        return true;
    }

    // Controller starting
    void ModelPredictiveController::starting(const ros::Time& time) { }

    // Controller running
    void ModelPredictiveController::update(const ros::Time& time, const ros::Duration& period)
    {
        double error = setpoint_ - joint_.getPosition();
        joint_.setCommand(gain_*error);
    }

    // Controller stopping
    void ModelPredictiveController::stopping(const ros::Time& time) { }

} //model_predictive_controller

PLUGINLIB_EXPORT_CLASS(model_predictive_controller::ModelPredictiveController, controller_interface::ControllerBase);
