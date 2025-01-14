//
// Created by and on 2025/1/14.
//
#include "cache_lidar.h"


cache_lidar::cache_lidar() {

    nh.param<int>("/cache_lidar/frame_num", frame_num, 5);
    nh.param<std::string>("/cache_lidar/sub_topic", sub_topic, "/livox/lidar");
    nh.param<std::string>("/cache_lidar/pub_topic", pub_topic, "/cache/lidar");


    sub_cloud_ = nh.subscribe(sub_topic, 1, &cache_lidar::cloudCallback, this);
    pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 10);
}

void cache_lidar::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *input_cloud);

    if(cache_cloud.size() == frame_num){
        pcl::PointCloud<PointT>::Ptr total_cloud;
        for(pcl::PointCloud<PointT>::Ptr cloud: cache_cloud){
            if(!cloud)return ;

            *total_cloud += *cloud;
        }
        sensor_msgs::PointCloud2 pub_cloud_msg;
        pcl::toROSMsg(*total_cloud, pub_cloud_msg);
        pub_cloud_msg.header = cloud_msg->header;
        pub_cloud_.publish(pub_cloud_msg);

        cache_cloud.pop_front();
    }
    cache_cloud.push_back(input_cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cache_lidar");
    cache_lidar cacheLidar;
    ros::spin();
    return 0;
}