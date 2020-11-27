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
#include <turtle_vis/TurtleClass.h>

namespace turtleSpace {

TurtleClass::TurtleClass() {
  //#>>>>TODO: INITIALIZE MEMBER VARIABLES
  count_mutex_ = PTHREAD_MUTEX_INITIALIZER;
  turtlePose_ = new Vector3D(0.0, 0.0, 0.0);
  turtlePose_desired_ = new Vector3D(0.0, 0.0, 0.0);
}
TurtleClass::~TurtleClass() {}

void TurtleClass::getPose(const turtle_vis::DesiredPose::ConstPtr &msg) {
  Vector3d local_msg;
  pthread_mutex_lock(&count_mutex_);
  //#>>>>TODO: COPY THE MSG TO A LOCAL VARIABLE
  local_msg = msg;
  pthread_mutex_unlock(&count_mutex_);

  ROS_INFO_STREAM("Vis Turtle Pose: " << turtlePose_.transpose());
}

bool TurtleClass::getDPose(turtle_vis::send_desired_pose::Request &req,
                           turtle_vis::send_desired_pose::Response &res) {
  Vector3d local_req; // correct data type???
  int local_res;
  pthread_mutex_lock(&count_mutex_);
  //#>>>>TODO:COPY THE REQUEST MSG TO A LOCAL VARIABLE
  local_req = req;
  local_res = res;
  pthread_mutex_unlock(&count_mutex_);

  ROS_INFO_STREAM("Desired Pose: " << turtlePose_desired_.transpose());

  res.reply = 1;

  return true;
}

void TurtleClass::setLocalPose(Vector3d &pose) {
  pthread_mutex_lock(&count_mutex_);
  turtlePose_ = pose;
  pthread_mutex_unlock(&count_mutex_);
}

Vector3d TurtleClass::getLocalPose() {
  Vector3d local;
  pthread_mutex_lock(&count_mutex_);
  local = turtlePose_;
  pthread_mutex_unlock(&count_mutex_);

  return local;
}

void TurtleClass::setLocalDesiredPose(Vector3d &pose) {
  pthread_mutex_lock(&count_mutex_);
  turtlePose_desired_ = pose;
  pthread_mutex_unlock(&count_mutex_);
}

Vector3d TurtleClass::getLocalDesiredPose() {
  Vector3d local;
  pthread_mutex_lock(&count_mutex_);
  local = turtlePose_desired_;
  pthread_mutex_unlock(&count_mutex_);

  return local;
}

const visualization_msgs::Marker
TurtleClass::createMarkerMesh(std::string frame_id, int id, int shape,
                              /*position*/
                              double x, double y, double z,
                              /*orientation*/
                              double q_w, double q_x, double q_y, double q_z,
                              /*scale*/
                              double s_x, double s_y, double s_z,
                              std::string meshFile) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "turtle_marker";
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = q_x;
  marker.pose.orientation.y = q_y;
  marker.pose.orientation.z = q_z;
  marker.pose.orientation.w = q_w;
  marker.scale.x = s_x;
  marker.scale.y = s_y;
  marker.scale.z = s_z;
  marker.mesh_resource = meshFile;

  marker.color.r = 0;
  marker.color.g = 0.7;
  marker.color.b = 0.5;
  marker.color.a = 1;
  marker.lifetime = ros::Duration();

  return marker;
}

} // namespace turtleSpace
