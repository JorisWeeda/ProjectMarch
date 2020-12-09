// Created by Herbert Van Even on 07-12-20. 15:54
#include <march_hardware/joint.h>
#include <march_hardware_interface/march_hardware_interface.h>
#include <march_joint_inertia_controller/inertia_estimator.h>
#include <march_joint_inertia_controller/inertia_estimator_node.h>
#include <rosbag/bag.h>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    //Retrieve velocity and effort
    const size_t index = 0; //getJoint() takes as an argument the index of the joint of jointlist you want t chose
    march::Joint& joint = march_robot_->getJoint(index); //retrieve a jointHandler object

    //Create a publisher node
    ros::init(argc, argv, "inertia_estimator_node");
    ros::NodeHandle nh;
    ros::Publisher joint_inertia_pub = nh.advertise<std_msgs::String>("joint_inertia", 1000);

    //Open bag file
    rosbag::Bag bag("inertia.bag", rosbag::bagmode::Write);

    //Set loop frequency
    ros::Rate loop_rate(250)
    ros::Duration period(0.004);

    while (ros::ok())
    {
        //Calculate inertia
        inertia_estimator_object.fillBuffers(joint.getVelocity(), joint.getTorque(), period);
        inertia_estimator_object.inertiaEstimate();
        joint_inertia = inertia_estimator_object.getJointInertia();

        //Publish inertia
        joint_inertia_msg.data = joint_inertia; //create message containing joint inertia
        joint_inertia_pub.publish(join_inertia_msg);
        ROS_INFO("The joint inertia is %f", joint_inertia_msg.data); //

        //Write inertia to bagfile
        bag.write("joint_inertia", ros::Time::now(), joint_inertia_msg);

        //Keep machinery running
        ros::spinOnce();

        //Control lop time
        loop_rate.sleep();
    }

    //close bag file
    bag.close();

}



