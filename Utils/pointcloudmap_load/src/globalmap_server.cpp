#include <pointcloudmap_load/globalmap_server.h>

GlobalmapServer::GlobalmapServer()
{
}

void GlobalmapServer::init(ros::NodeHandle nh_) {
    globalmap_pcd_ = nh_.param<std::string>("globalmap_pcd","/");
    downsample_or_not_ = nh_.param<bool>("downsample_or_not", true);
    downsample_resolution_ = nh_.param<double>("downsample_resolution", 0.1);
    
    globalmap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    addmap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/addmap", 5, true);
    globalmap_.reset(new pcl::PointCloud<pcl::PointXYZI>());  

    odom_x1 = nh_.param<double>("/globalmap_server/odom_x1",0.0);
    odom_y1 = nh_.param<double>("/globalmap_server/odom_y1",0.0);
    odom_x2 = nh_.param<double>("/globalmap_server/odom_x2",0.0);
    odom_y2 = nh_.param<double>("/globalmap_server/odom_y2",0.0);
    odom_z1 = nh_.param<int>("/globalmap_server/odom_z1",0);
    odom_z2 = nh_.param<int>("/globalmap_server/odom_z2",0);
    if(globalmap_pcd_ == "/")
    {
        ROS_ERROR("Please enter the global map path.");
    }
    
    if( pcl::io::loadPCDFile(globalmap_pcd_, *globalmap_) != 0 )
    {
        ROS_ERROR("Please enter the right global map path.");
    }
    else
    {
        ROS_INFO("\033[1;32m---->\033[0m load Globalmap successfully.");

        if( downsample_or_not_ == true)
        {
            // downsample globalmap
            boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI> > voxelgrid(new pcl::VoxelGrid<pcl::PointXYZI>());
            voxelgrid->setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
            voxelgrid->setInputCloud(globalmap_);

            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
            voxelgrid->filter(*filtered);

            globalmap_filtered_ = filtered;

            // pcl::io::savePCDFileBinary(globalmap_pcd_ + "_filter.pcd", *filtered);

            globalmap_filtered_->header.frame_id = "world";
            globalmap_pub_.publish(globalmap_filtered_);
            ROS_INFO("\033[1;32m---->\033[0m published downsampled Globalmap.");
        }
        else
        {
            globalmap_->header.frame_id = "world";
            globalmap_pub_.publish(globalmap_);
            ROS_INFO("\033[1;32m---->\033[0m published not downsampled Globalmap.");
        }

    }
}

void GlobalmapServer::loop_pub(pcl::PointCloud<pcl::PointXYZI> input) {

    // ROS_INFO("\033[1;32m---->\033[0m load Globalmap successfully.");
    int input_size = input.size();
    if(downsample_or_not_ == true)
    {
        std::cout << "before: " << globalmap_filtered_->size()  << std::endl;
        std::cout << "input_size: " << input_size  << std::endl;

        if(input_size > 0) {
            ROS_INFO("Get input.");    
            // for(int index = 0; index < input_size; index++) {

            //         pcl::PointXYZI add_pcl = input.points[index];
            //         globalmap_filtered_->points.push_back(add_pcl);

            //     }

            for(int index = 0; index < globalmap_filtered_->size(); index++) {
                
                pcl::PointXYZI add_pcl = globalmap_filtered_->points[index];
                input.points.push_back(add_pcl);
            }

            input.header.frame_id = "world";
            addmap_pub_.publish(input);
            globalmap_filtered_->header.frame_id = "world";
            globalmap_pub_.publish(input);
        }

        std::cout << "after: " << globalmap_filtered_->size() << std::endl;
        // pcl::io::savePCDFileBinary(globalmap_pcd_ + "_filter.pcd", *filtered);


        // ROS_INFO("\033[1;32m---->\033[0m published downsampled Globalmap.");
    }
    else
    {
        globalmap_->header.frame_id = "world";
        globalmap_pub_.publish(globalmap_);
    
        if(input_size > 0) {
            ROS_INFO("Get input.");    
            for(int index = 0; index < globalmap_filtered_->size(); index++) {
                
                pcl::PointXYZI add_pcl = globalmap_filtered_->points[index];
                input.points.push_back(add_pcl);
            }

            input.header.frame_id = "world";
            addmap_pub_.publish(input);
            globalmap_filtered_->header.frame_id = "world";
            globalmap_pub_.publish(input);
        }
        // ROS_INFO("\033[1;32m---->\033[0m published not downsampled Globalmap.");
    }
}

GlobalmapServer::~GlobalmapServer(){

}