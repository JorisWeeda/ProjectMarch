
#ifndef MARCH_HARDWARE_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_HARDWARE_MODEL_PREDICTIVE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace model_predictive_controller {

    class ModelPredictiveController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
        void update(const ros::Time& time, const ros::Duration& period);
        void starting(const ros::Time& time);
        void stopping(const ros::Time& time);

    private:
        hardware_interface::JointHandle joint_;
        const double gain_ = 30;
        const double setpoint_ = 1.0;
    };
//    PLUGINLIB_DECLARE_CLASS(march_acado_mpc, ModelPredictiveController, model_predictive_controller::ModelPredictiveController, controller_interface::ControllerBase);
} //model_predictive_controller

#endif //MARCH_HARDWARE_MODEL_PREDICTIVE_CONTROLLER_H
