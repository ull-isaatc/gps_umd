/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

using namespace gps_common;

static ros::Publisher odom_pub, cov_marker_pub;
std::string frame_id, child_frame_id;
double rot_cov;
double rotacion_incremental, tmp_rot, tmp_rot_mod;
tf::Quaternion tmp_quat;

void callback(const gps_common::GPSFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_WARN("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    ROS_WARN("Fix time is 0.");
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = northing;
    odom.pose.pose.position.y = -easting;
//    odom.pose.pose.position.z = fix->altitude;
    odom.pose.pose.position.z = 0;

    tmp_rot = tmp_rot + 0.1;
    tmp_rot_mod = fmod(tmp_rot, 3.1415);

    //tmp_quat = tf::createQuaternionFromRPY(0, 0, 0);
    tmp_quat = tf::createQuaternionFromRPY(0, 0, -fix->track * RADIANS_PER_DEGREE);
    tf::quaternionTFToMsg(tmp_quat, odom.pose.pose.orientation);


//    odom.pose.pose.orientation.x = tmp_quat.getX;
//    odom.pose.pose.orientation.y = tmp_quat.getY;
//    odom.pose.pose.orientation.z = tmp_quat.getZ;
//    odom.pose.pose.orientation.w = tmp_quat.getW;
    //ROS_INFO("tmp_quat =  %f, %f, %f, %f\n", tmp_quat);

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

//    ROS_DEBUG("UTM publishing at time %f", ros::Time::now().toSec());
    ROS_DEBUG("GPS UTM odometry publishing with std. deviation in m. (x: %f y: %f, z: %f)", fix->position_covariance[0], fix->position_covariance[4], fix->position_covariance[8]);

    odom_pub.publish(odom);

    visualization_msgs::Marker marker;
    marker.header.frame_id = odom.header.frame_id;
    marker.header.stamp = fix->header.stamp;
    marker.ns = "utm_odometry_node";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = odom.pose.pose.position.x;
    marker.pose.position.y = odom.pose.pose.position.y;
    marker.pose.position.z = odom.pose.pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = fix->position_covariance[0];
    marker.scale.y = fix->position_covariance[4];
    //marker.scale.z = 0.01;
    marker.scale.z = fix->position_covariance[8];


    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.5f;

    marker.lifetime = ros::Duration();

    // Publish the marker
    cov_marker_pub.publish(marker);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<double>("rotacion_incremental", rotacion_incremental, 0);
  tmp_rot = rotacion_incremental;


  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1);

  cov_marker_pub = node.advertise<visualization_msgs::Marker>("odom_covariance", 1);

  ros::Subscriber fix_sub = node.subscribe("fix", 1, callback);

  ros::spin();
}

