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
//INCLUDE TURTLE CLASS HEADER
#include <turtle_vis/TurtleClass.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "turtle_visualization",
            ros::init_options::AnonymousName);

  ROS_INFO_STREAM("**Publishing turtle position for rviz.");

  ros::NodeHandle n;
  ros::Rate r(60);

  static tf::TransformBroadcaster br;

  tf::Transform transform;

  ros::Publisher turtle_marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //INSTANTIATE THE TURTLE CLASS WITH THE VARIABLE my_turtle;
  turtleSpace::TurtleClass my_turtle;

  visualization_msgs::Marker turtle3D;

  turtle3D = turtleSpace::TurtleClass::createMarkerMesh(
      "turtle", 12345, visualization_msgs::Marker::MESH_RESOURCE,
      /*pos xyz:*/ 0.0, 0.024, -0.021,
      /*orientation quatern wxyz*/ 0, 0, 0, 1,
      /*scale s_x s_y s_z*/ 0.025, 0.025, 0.025,
      "package://turtle_vis/meshes/turtle2.dae");

  // INIT A SUBSCRIBER TO THE TOPIC "turtle_control" USING THE TURTLE CLASS
  // OBJECT AND THE CALLBACK METHOD
  ros::Subscriber sub = n.subscribe(
      "turtle_control", 100,
      &turtleSpace::TurtleClass::getPose /*DEFINE THE CALLBACK FUNCTION FROM
                                            THE METHOD OF THE CLASS*/
      ,
      &my_turtle /* DEFINE THE INSTANCE OF THE CLASS*/);

  tf::Quaternion qtf;

  Vector3d turtlePose;

  turtlePose << 1, 0, 0;

  // Target
  Vector3d turtlePose_local;
  my_turtle.setLocalPose(turtlePose);
  turtlePose_local = turtlePose;

  while (ros::ok()) {

    // GetPose
    turtlePose_local = my_turtle.getLocalPose();

    // Update pose and visualize the turtle marker
    qtf.setRPY(0, 0, turtlePose_local(2));
    transform.setOrigin(
        tf::Vector3(turtlePose_local(0), turtlePose_local(1), 0));

    transform.setRotation(qtf);
    //PUBLISH THE TF FOR THE TURTLE USING transform, parent, child, and ros::Time::now()
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "/world", "/turtle"));

    // PUBLISH THE TURTLE MARKER
    turtle_marker_pub.publish(
        turtle3D /*USE THE MSG WITH TYPE MARKER*/);

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
