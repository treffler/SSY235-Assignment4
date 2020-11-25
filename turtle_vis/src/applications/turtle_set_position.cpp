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
#include <iostream>
#include <fstream>
#include <pthread.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

/*********************************************************************
 * SEVICES AND MESSAGES
 * ******************************************************************/
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

using namespace Eigen;


int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtle_set_position_node",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Client turtle desired position");

    ros::NodeHandle n;
    ros::Rate r(60);

    //INITIALIZE THE CLIENT
    ros::ServiceClient client=
        n.serviceClient<TURTLE_SERVICE_TYPE/*//#>>>>TODO: DEFINE THE SERVICE TYPE*/>("TurtlePose");

    ////#>>>>TODO: DEFINE A MSG VARIABLE FOR THE SERVICE MESSAGE
    XXXXXXXXX desired_pose_msg;

    std::string keyb_in;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion qtf;

    while(ros::ok())
    {

        std::vector<double> key_vals;

        ROS_INFO_STREAM(
              "Give me the desired position of the turtle: x,y,theta");
        std::cin>>keyb_in;

        ROS_INFO_STREAM("typed: "<<keyb_in);
        ////#>>>>TODO:GET THE VALUES FROM THE TERMINAL AND SAVE THEM IN A LOCAL VARIABLE. YOU WILL GET X,Y AND THETA

        ////#>>>>TODO:CREATE THE MESSAGE WITH THE LOCAL VARIABLE (desired_pose_msg)


        ////#>>>>TODO:COMPUTE THE POSITION AND ORIENTATION OF THE TF FOR THE DESIRED POSITION

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

        delete[] writable;
    }



    return 0;
}
