/*********************************************************************
 * Compiler:         >gcc 4.6.3
 *
 * Company:          Chalmers University of Technology
 *
 * Author:           Emmanuel Dean (deane@chalmers.se)
 *                   Karinne Ramirez (karinne@chalmers.se)
 *
 * Compatibility:    Ubuntu 18.04 64bit (ros melodic)
 *
 * Software Version: V0.1
 *
 * Created:          01.11.2020
 *
 * Comment:          turtle connection and visualization (Sensor and Signals)
 *
 ********************************************************************/

/*********************************************************************
 * STD INCLUDES
 ********************************************************************/
#include <fstream>
#include <iostream>
#include <pthread.h>

/*********************************************************************
 * ROS INCLUDES
 ********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*********************************************************************
 * EIGEN INCLUDES
 ********************************************************************/
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

/*********************************************************************
 * SEVICES AND MESSAGES
 * ******************************************************************/
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

using namespace Eigen;

int main(int argc, char **argv) {

  ros::init(argc, argv, "turtle_set_position_node",
            ros::init_options::AnonymousName);

  ROS_INFO_STREAM("**Client turtle desired position");

  ros::NodeHandle n;
  ros::Rate r(60);

  // INITIALIZE THE CLIENT
  ros::ServiceClient client = n.serviceClient<
      turtle_vis::send_desired_pose /* DEFINE THE SERVICE TYPE*/>(
      "TurtlePose");

  // DEFINE A MSG VARIABLE FOR THE SERVICE MESSAGE
  turtle_vis::send_desired_pose desired_pose_msg;

  std::string keyb_in;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion qtf;

  while (ros::ok()) {

    std::vector<double> key_vals;

    ROS_INFO_STREAM("Give me the desired position of the turtle: x,y,theta");
    std::cin >> keyb_in;

    ROS_INFO_STREAM("typed: " << keyb_in);
    // GET THE VALUES FROM THE TERMINAL AND SAVE THEM IN A LOCAL
    /// VARIABLE. YOU WILL GET X,Y AND THETA
    char *writable = new char[keyb_in.size() + 1];
    std::copy(keyb_in.begin(), keyb_in.end(), writable);
    writable[keyb_in.size()] = '\0';
    char *p = strtok(writable, ",");
    while (p) {
      key_vals.push_back(atof(p));
      p = strtok(NULL, ",");
    }

    // CREATE THE MESSAGE WITH THE LOCAL VARIABLE (desired_pose_msg)
    desired_pose_msg.request.desiredPose.x = key_vals[0];
    desired_pose_msg.request.desiredPose.y = key_vals[1];
    desired_pose_msg.request.desiredPose.theta = key_vals[2];

    // COMPUTE THE POSITION AND ORIENTATION OF THE TF FOR THE DESIRED
    /// POSITION

    qtf.setRPY(0, 0, key_vals[2]); // USE THETA VARIABLE);
    transform.setOrigin(tf::Vector3(key_vals[0] /* USE X VARIABLE*/,
                                    key_vals[1] /* USE Y VARIABLE*/,
                                    0));
    transform.setRotation(qtf);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world",
                                          "/turtle_desired"));

    if (client.call(desired_pose_msg)) // CALL THE CLIENT WITH desired_pose_msg)
    {
      ROS_INFO_STREAM("desired pose: x = "
                      << desired_pose_msg.request.desiredPose.x << " y = "
                      << desired_pose_msg.request.desiredPose.y << " theta = "
                      << desired_pose_msg.request.desiredPose
                             .theta); // PRINT OUT THE MESSAGE);
    } else {
      ROS_ERROR_STREAM("Failed to call the service 'TurtlePose'");
      return 1;
    }

    delete[] writable;
  }

  return 0;
}
