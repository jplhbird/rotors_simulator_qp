/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  Eigen::Vector3d desired_position(0.0, 0.0, 2.0);
  double desired_yaw = 0.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());
 // trajectory_pub.publish(trajectory_msg);




  //test:

  ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 10);


  geometry_msgs::PoseStamped posecmd;

  posecmd.pose.position.x = 1;
  posecmd.pose.position.y = 2;
  posecmd.pose.position.z = 3;

  posecmd.pose.orientation.x = 0;
  posecmd.pose.orientation.y = 0;
  posecmd.pose.orientation.z = 0;
  posecmd.pose.orientation.w = 0;

//  for ( int i = 0; i<100000000; i++)
//  {
//	  posecmd.pose.position.z = posecmd.pose.position.z + 0.001;
//
//	  pose_pub.publish(posecmd);
//  }

  // %Tag(LOOP_RATE)%
     ros::Rate loop_rate(0.01);
  // %EndTag(LOOP_RATE)%

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
  // %Tag(ROS_OK)%
    int count = 0;
    while (ros::ok())
    {
    	/**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
  // %Tag(PUBLISH)%
    	//pose_pub.publish(posecmd);
  // %EndTag(PUBLISH)%
//    	ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
//    	           nh.getNamespace().c_str(),
//				   posecmd.pose.position.x,
//				   posecmd.pose.position.y,
//				   posecmd.pose.position.z);

  // %Tag(SPINONCE)%
      ros::spinOnce();
  // %EndTag(SPINONCE)%

  // %Tag(RATE_SLEEP)%
      loop_rate.sleep();
  // %EndTag(RATE_SLEEP)%
      ++count;
    }







  ros::spin();
}
