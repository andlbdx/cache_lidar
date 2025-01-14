//
// Created by and on 2025/1/14.
//

#ifndef CACHE_LIDAR_CACHE_H
#define CACHE_LIDAR_CACHE_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <deque>
typedef pcl::PointXYZI PointT;

class cache_lidar {
public:
    cache_lidar();

    void cloudCallback( const sensor_msgs::PointCloud2::ConstPtr &input_cloud);

private:
    int frame_num; // 缓存帧数
    std::string sub_topic;
    std::string pub_topic;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_cloud_;
    ros::NodeHandle nh;

    std::deque<pcl::PointCloud<PointT>::Ptr> cache_cloud;

};


#endif //CACHE_LIDAR_CACHE_H
