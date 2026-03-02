#ifndef GLOBALMAP_SERVER_H
#define GLOBALMAP_SERVER_H

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

class GlobalmapServer
{
    public:

    GlobalmapServer();

    ~GlobalmapServer();
    void loop_pub(pcl::PointCloud<pcl::PointXYZI> input);
    void init(ros::NodeHandle nh_);

    public:
        // ros::NodeHandle nh_;
        std::string globalmap_pcd_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap_;
        double odom_x1, odom_y1, odom_x2,odom_y2;
        int odom_z1,odom_z2;
    private:
        
        // ros::NodeHandle private_nh_;
        ros::Publisher globalmap_pub_;
        ros::Publisher addmap_pub_;
        double downsample_or_not_;
        double downsample_resolution_;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap_filtered_;
};

#endif