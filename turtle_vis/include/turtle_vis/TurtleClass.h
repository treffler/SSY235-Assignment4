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
#ifndef TURTLECLASS_H
#define TURTLECLASS_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>


using namespace Eigen;

namespace turtleSpace
{
/*!
 * \brief The Turtle class
 *
 * This class is a simple example to set and get member variables with
 * mutex control.
 */
class TurtleClass
{
//Member Variables
private:
  Vector3d turtlePose_; //!< current turtle pose (x,y,theta)
  Vector3d turtlePose_desired_; //!< desired turtle pose (x,y,theta)

private:
  pthread_mutex_t count_mutex_; //!<mutex to control the read/write acces

//Member functions
public:
  /*!
   * \brief TurtleClass
   *    Default constructor.
   */
  TurtleClass();
  ~TurtleClass();

  /*!
   * \brief Gets the current turtle pose.
   *
   * \param msg
   *    current pose in turtle_vis::DesiredPose format.
   */
  void getPose(const turtle_vis::DesiredPose::ConstPtr &msg);

  /*!
   * \brief Callback function for the service "TurtlePose".
   *
   * \return
   *    true on sucess
   */
  bool getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res);

  /*!
   * \brief Set the current turtle pose.
   *
   * \param
   *    Current pose of the turtle as a 3D vector [x,y,theta]
   */
  void setLocalPose(Eigen::Vector3d& pose);

  /*!
   * \brief Get the current turtle pose.
   *
   * \return
   *    Returns the current pose of the turtle as a 3D vector [x,y,theta]
   */
  Vector3d getLocalPose();

  /*!
   * \brief Set the desired turtle pose.
   *
   * \param
   *    Desired pose of the turtle as a 3D vector [x,y,theta]
   */
  void setLocalDesiredPose(Eigen::Vector3d& pose);

  /*!
   * \brief Get the desired turtle pose.
   *
   * \return
   *    Returns the desired pose of the turtle as a 3D vector [x,y,theta]
   */
  Vector3d getLocalDesiredPose();

  /*!
   * \brief creates a marker object
   * \param frame_id reference tf name to connect with the marker
   * \param id marker id
   * \param shape type of marker shape, e.g. CUBE, CYLINDER, MESH_RESOURCE
   * \param x position in x relative to the reference tf
   * \param y position in x relative to the reference tf
   * \param z position in x relative to the reference tf
   * \param q_w orientation in quaternion w relative to the reference tf
   * \param q_x orientation in quaternion x relative to the reference tf
   * \param q_y orientation in quaternion y relative to the reference tf
   * \param q_z orientation in quaternion z relative to the reference tf
   * \param s_x marker scale in x axis
   * \param s_y marker scale in y axis
   * \param s_z marker scale in z axis
   * \param meshFile filepath for the mesh (dae/stl)
   * \return marker object
   */
  static const visualization_msgs::Marker createMarkerMesh(
      std::string frame_id,
      int id, int shape,
      /*position*/
      double x, double y, double z,
      /*orientation*/
      double q_w, double q_x,
      double q_y, double q_z,
      /*scale*/
      double s_x, double s_y, double s_z,
      std::string meshFile);

};


}

#endif // TURTLECLASS_H
