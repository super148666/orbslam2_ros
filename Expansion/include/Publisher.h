//
// Created by cz on 27/03/19.
//

#ifndef ORBSLAM2_ROS_PUBLISHER_H
#define ORBSLAM2_ROS_PUBLISHER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Converter.h"
#include "Map.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Publisher {
public:
    explicit Publisher(ORB_SLAM2::Map *map, ros::NodeHandle n);

    void publish(cv::Mat Tcw, ros::Time time_stamp);

private:
    void _publish_pose(cv::Mat Tcw);

    void _publish_pointcloud();

    void _publish_tf();

    tf2::Transform _pose_to_tf(cv::Mat pose);

    pcl::PointXYZ _pose_to_point(cv::Mat pose);

    PointCloud::Ptr _map_to_pcl();

    void ReadParameters();

    template<class T>
    void getParam(const std::string &param_name, T &container, const T &defaultVal);

    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher pcl_pub;
    tf::TransformBroadcaster *tran_broadcaster;
    geometry_msgs::TransformStamped cam_trans_msg;
    geometry_msgs::TransformStamped base_trans_msg;
    tf2::Transform cam_trans;
    tf2::Transform base_trans;
    tf2::Quaternion cam_trans_q;
    ORB_SLAM2::Map *mpMap;

    ros::Time last_time_stamp;
    std::string package_name;

    bool pub_pose;
    std::string pose_topic_name;
    std::string camera_link_frame_id;
    std::string map_link_frame_id;
    bool pub_tf;
    std::string base_link_frame_id;
    bool pub_pcl;
    std::string pointcloud_topic_name;
    bool is_publish;

    float scale_factor;
};


#endif //ORBSLAM2_ROS_PUBLISHER_H
