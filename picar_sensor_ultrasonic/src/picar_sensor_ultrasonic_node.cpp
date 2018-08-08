#include <ros/ros.h>
#include <ros/package.h>
#include "../include/picar_sensor_ultrasonic/picar_sensor_ultrasonic.h"


int main (int argc, char** argv)
{
  ros::init (argc, argv, "ipa_nav_state_observer");
  
  // New instance, constructor initializes all the path segments
  SensorUltrasonic node;
  node.configure();

  ros::Rate loop_rate(0.2); // Hz
  while (ros::ok())
  {
    node.update();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
