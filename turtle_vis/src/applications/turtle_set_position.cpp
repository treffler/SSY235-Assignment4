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
      turtle_vis::send_desired_pose /*//#>>>>TODO: DEFINE THE SERVICE TYPE*/>(
      "TurtlePose");

  ////#>>>>TODO: DEFINE A MSG VARIABLE FOR THE SERVICE MESSAGE
  Vector3d desired_pose_msg;

  std::string keyb_in;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion qtf;

  while (ros::ok()) {

    std::vector<double> key_vals;

    ROS_INFO_STREAM("Give me the desired position of the turtle: x,y,theta");
    std::cin >> keyb_in;

    ROS_INFO_STREAM("typed: " << keyb_in);
    ////#>>>>TODO:GET THE VALUES FROM THE TERMINAL AND SAVE THEM IN A LOCAL
    /// VARIABLE. YOU WILL GET X,Y AND THETA
    std::string temp;
    for (int i = 0; i < keyb_in.size(); i++) {
      if (isdigit(keyb_in[i])) {
        temp += keyb_in[i];
      } else if (keyb_in[i] == ',') {
        key_vals.push_back(atof(temp.c_str()));
        temp = "";
      }
    }
    key_vals.push_back(atof(temp.c_str()));

    ////#>>>>TODO:CREATE THE MESSAGE WITH THE LOCAL VARIABLE (desired_pose_msg)
    desired_pose_msg.x = key_vals[0];
    desired_pose_msg.y = key_vals[1];
    desired_pose_msg.z = key_vals[2];

    ////#>>>>TODO:COMPUTE THE POSITION AND ORIENTATION OF THE TF FOR THE DESIRED
    /// POSITION

        qtf.setRPY(0,0,////#>>>>TODO:USE THETA VARIABLE);
        transform.setOrigin(tf::Vector3(/*//#>>>>TODO:USE X VARIABLE*/, /*//#>>>>TODO:USE Y VARIABLE*/, 0));
        transform.setRotation(qtf);

        br.sendTransform(tf::StampedTransform(
                           transform,ros::Time::now(),
                           "/world","/turtle_desired"));

        if(//#>>>>TODO:CALL THE CLIENT WITH desired_pose_msg)
        {
            ROS_INFO_STREAM(//#>>>>TODO:PRINT OUT THE MESSAGE);
        }
        else
        {
      ROS_ERROR_STREAM("Failed to call the service 'TurtlePose'");
      return 1;
        }

        // delete[] writable;
  }

  return 0;
}
