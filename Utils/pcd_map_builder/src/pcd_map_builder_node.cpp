#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class PcdMapBuilder
{
public:
  PcdMapBuilder(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
    pnh.param<std::string>("save_path", save_path_, "map.pcd");

    map_.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    sub_ = nh.subscribe(cloud_topic_, 5000, &PcdMapBuilder::cloudCallback, this);

    ROS_INFO("[pcd_map_builder] accumulating topic %s -> %s (no downsampling)",
             cloud_topic_.c_str(), save_path_.c_str());
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    pcl::fromROSMsg(*msg, cloud);
    *map_ += cloud;
    received_++;
    if (received_ % 100 == 0)
      ROS_INFO("[pcd_map_builder] received %d clouds, total %zu points",
               received_, map_->size());
  }

  void save()
  {
    map_->width = map_->size();
    map_->height = 1;
    map_->is_dense = false;
    map_->header.frame_id = "world";
    if (map_->empty())
    {
      ROS_ERROR("[pcd_map_builder] nothing accumulated, skip saving.");
      return;
    }
    if (pcl::io::savePCDFileBinary(save_path_, *map_) == 0)
      ROS_INFO("[pcd_map_builder] saved %zu points to %s", map_->size(), save_path_.c_str());
    else
      ROS_ERROR("[pcd_map_builder] failed to save %s", save_path_.c_str());
  }

private:
  ros::Subscriber sub_;
  std::string cloud_topic_, save_path_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr map_;
  int received_ = 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_map_builder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PcdMapBuilder builder(nh, pnh);
  ros::spin();
  builder.save();
  return 0;
}
