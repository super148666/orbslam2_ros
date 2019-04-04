#include <utility>

//
// Created by cz on 27/03/19.
//

#include <Expansion/include/Publisher.h>

Publisher::Publisher(ORB_SLAM2::Map *map, ros::NodeHandle n) : n(n), mpMap(map) {
    ReadParameters();
    scale_factor = 8;
}

void Publisher::ReadParameters() {
    package_name = "orbslam2_ros";
    std::string pkg_path = ros::package::getPath(package_name) + "/";

    getParam("publish_pose", pub_pose, true);

    if (pub_pose) {
        getParam("pose_topic_name", pose_topic_name, std::string("/pose"));
        pose_pub = n.advertise<nav_msgs::Odometry>(pose_topic_name, 1);
    }

    getParam("publish_pointcloud", pub_pcl, true);

    if (pub_pcl) {
        getParam("pointcloud_topic_name", pointcloud_topic_name, std::string("/points2"));
        pcl_pub = n.advertise<PointCloud>(pointcloud_topic_name, 1);
    }

    getParam("publish_transform", pub_tf, true);

    is_publish = pub_tf | pub_pose | pub_pcl;

    if (is_publish) {
        getParam("camera_link_frame_id", camera_link_frame_id, std::string("camera_link"));
        getParam("base_link_frame_id", base_link_frame_id, std::string("base_link"));
        getParam("map_link_frame_id", map_link_frame_id, std::string("visual_odom"));
        tran_broadcaster = new tf::TransformBroadcaster();
        cam_trans_msg.header.frame_id = camera_link_frame_id;
        cam_trans_msg.child_frame_id = base_link_frame_id;
        tf2::Quaternion quaternion;
        quaternion.setRPY(M_PI_2, 0.0, M_PI_2);
        cam_trans = tf2::Transform(quaternion, tf2::Vector3(0.0, 0.0, 0.0));
        cam_trans_msg.transform = tf2::toMsg(cam_trans);

        base_trans_msg.header.frame_id = base_link_frame_id;
        base_trans_msg.child_frame_id = camera_link_frame_id;
        quaternion.setRPY(-M_PI_2, 0.0, -M_PI_2);
        base_trans = tf2::Transform(quaternion, tf2::Vector3(0.0, 0.0, 0.0));
        base_trans_msg.transform = tf2::toMsg(base_trans);

    }
}

void Publisher::publish(cv::Mat Tcw, ros::Time time_stamp) {
    if (!is_publish) {
        return;
    }

    last_time_stamp = time_stamp;

    if (pub_tf) {
        _publish_tf();
    }

    if (pub_pose) {
        _publish_pose(std::move(Tcw));
    }

    if (pub_pcl) {
        _publish_pointcloud();
    }
}

void Publisher::_publish_pose(cv::Mat Tcw) {
    if (!Tcw.empty()) {
        nav_msgs::Odometry position;
        position.header.frame_id = map_link_frame_id;
        position.child_frame_id = base_link_frame_id;
        position.header.stamp = last_time_stamp;
        auto transform = _pose_to_tf(Tcw);
        tf2::toMsg(transform, position.pose.pose);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header = position.header;
        odom_trans.child_frame_id = position.child_frame_id;

        odom_trans.transform = tf2::toMsg(transform);
        tran_broadcaster->sendTransform(odom_trans);
        pose_pub.publish(position);
    }
}

void Publisher::_publish_pointcloud() {
    auto pcl_msg = _map_to_pcl();
    pcl_conversions::toPCL(last_time_stamp, pcl_msg->header.stamp);
    pcl_pub.publish(pcl_msg);
}

void Publisher::_publish_tf() {
    base_trans_msg.header.stamp = last_time_stamp;
    tran_broadcaster->sendTransform(base_trans_msg);
}

template<class T>
void Publisher::getParam(const std::string &param_name, T &container, const T &defaultVal) {
    n.param("/" + package_name + "/" + param_name, container, defaultVal);
}

tf2::Transform Publisher::_pose_to_tf(cv::Mat pose) {
    cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * pose.rowRange(0, 3).col(3);
    std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(scale_factor * twc.at<float>(0, 2), -scale_factor * twc.at<float>(0, 0),
                                     -scale_factor * twc.at<float>(0, 1)));

    tf2::Quaternion quaternion(q[0], q[1], q[2], q[3]);
    transform.setRotation(quaternion);
    return transform;
}

PointCloud::Ptr Publisher::_map_to_pcl() {
    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = map_link_frame_id;
    msg->height = 1;
    auto map_points = mpMap->GetAllMapPoints();
    for (auto &map_point : map_points) {
        auto pose = map_point->GetWorldPos();
        msg->points.push_back(_pose_to_point(pose));
    }
    msg->width = static_cast<uint32_t>(msg->points.size());
    return msg;
}

pcl::PointXYZ Publisher::_pose_to_point(cv::Mat pose) {
    return {scale_factor * pose.at<float>(0, 2), -scale_factor * pose.at<float>(0, 0),
            -scale_factor * pose.at<float>(0, 1)};
}



