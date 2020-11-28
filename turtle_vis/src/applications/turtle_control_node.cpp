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
 * CUSTOM CLASS
 * ******************************************************************/
#include <turtle_vis/TurtleClass.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "turtle_control", ros::init_options::AnonymousName);

  ROS_INFO_STREAM("**Publishing turtle control..");

  ros::NodeHandle n;
  ros::Rate r(60);

  // Service
  turtleSpace::TurtleClass turtleF;
  ros::ServiceServer service = n.advertiseService(
      "TurtlePose",
      &turtleSpace::TurtleClass::getDPose(), //#>>>>TODO: DEFINE THE
                                             // CALLBACK FUNCTION -> correct
                                             // function?
      &turtleF);
  // CALL SERVICE FROM TERMINAL//
  //    rosservice call /TurtlePose '{p: [0.5, 0.0, 3.0]}'
  //    rosservice call /TurtlePose "{p: {x: 1.5, y: 1.0, theta: 0.0}}"
  // DON'T FORGET TO SOURCE THE WORKSPACE BEFORE CALLING THE SERVICE

  // Topic
  ros::Publisher desired_pose_pub =
      n.advertise<turtle_vis::float64 /*#>>>>TODO: DEFINE THE MSG TYPE*/>(
          "turtle_control", 100);

  Matrix3d Kp;

  ros::Time ti, tf;

  ti = ros::Time::now();

  // Proportional Gain

  double p = 0.0;

  //#>>>>TODO: LOAD p_gain FROM THE ROS PARAMETER SERVER

  // LOAD p_gain FROM THE ROS PARAMETER SERVER
  if (ros::param::has("control_gain/p_gain" /*/#>>>>TODO: DEFINE PARAMETER*/)) {
    ros::param::get(
        "control_gain/p_gain" /*/#>>>>TODO: DEFINE PARAMETER, SAME AS LINE 88*/,
        p);
    ROS_INFO_STREAM("p gain= " << p);
  } else {
    ROS_ERROR_STREAM("param: control_gain/p_gain, doesn't exist");
    return 1;
  }

  // Proportional Gain Matrix
  Kp << p, 0, 0, 0, p, 0, 0, 0, p;

  ROS_INFO_STREAM("Kp= \n" << Kp);

  Vector3d turtlePose, turtlePose_old, turtleVel;
  Vector3d error;
  double dt;

  turtlePose << 1, 0, 0;
  turtlePose_old = turtlePose;
  turtleVel << 0, 0, 0;

  // Target
  Vector3d turtlePose_desired_local;
  ////#>>>>TODO: INITIALIZE THE DESIRED POSE VARIABLE OF THE CLASS TURTLE
  turtleF.setLocalDesiredPose(turtlePose);

  turtlePose_desired_local = turtlePose;

  // CREATE A DESIRED POSE MSG VARIABLE
  turtle_vis::float64 desired_pose_msg; //#>>>>TODO:DEFINE THE MSG TYPE

  while (ros::ok()) {

    tf = ros::Time::now();

    // Delta Time
    dt = tf.toSec() - ti.toSec();

    ////#>>>>TODO: Get Desired Pose from the class variable
    turtlePose_desired_local = turtleF.getLocalDesiredPose();

    // Control
    ////#>>>>TODO:COMPUTE THE ERROR BETWEEN CURRENT POSE AND DESIRED
    error = turtlePose_old - turtlePose_desired_local;
    turtleVel = -Kp * error;

    ////#>>>>TODO:COMPUTE THE NEW TURTLE POSE
    turtlePose = turtlePose_old + turtleVel * dt; // USE SIMPLE INTEGRATION

    turtleF.setLocalPose(turtlePose);

    // Publish Data
    ////#>>>>TODO:SET THE MSG VARIABLE WITH THE NEW TURTLE POSE
    desired_pose_msg.data(turtlePose_desired_local);
    desired_pose_pub.publish(desired_pose_msg);

    // SET THE HISTORY
    turtlePose_old = turtlePose;

    ti = tf;

    // ROS::SPIN IS IMPORTANT TO UPDATE ALL THE SERVICES AND TOPICS
    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
