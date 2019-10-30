// Copyright 2018 Project March.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "march_shared_resources/Sound.h"
#include "march_sound_scheduler/Scheduler.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_sound_scheduler");
  ros::NodeHandle n;

  ros::Publisher soundPub = n.advertise<sound_play::SoundRequest>("/robotsound", 0);

  Scheduler scheduler;

  ros::Subscriber soundSub = n.subscribe("/march/sound/schedule", 10, &Scheduler::scheduleMsg, &scheduler);
  while (0 == soundPub.getNumSubscribers())
  {
    ROS_DEBUG("Waiting on sound play topic");
    ros::Duration(0.1).sleep();
  }

  ros::Rate rate(10);
  while (ros::ok())
  {
    scheduler.spin();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
