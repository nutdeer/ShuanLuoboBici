#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class CloudWebFilter
{
    public:
        CloudWebFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            // 构造函数定义
            pnh.param<std::string>("input_topic", input_topic_, "/cloud_registered");
            pnh.param<std::string>("output_topic", output_topic_, "/cloud_registered_web");

            pnh.param<double>("voxel_size", voxel_size_, 0.2);
            pnh.param<double>("publish_rate", publish_rate_, 5.0);  // Hz

            pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
            sub_ = nh.subscribe(input_topic_, 1, &CloudWebFilter::cloudCallback, this);

            last_publish_time_ = ros::Time(0);

            ROS_INFO("[cloud_web_filter] %s -> %s, voxel=%.3f, rate=%.1f Hz",
                    input_topic_.c_str(), output_topic_.c_str(), voxel_size_, publish_rate_);
            
            if(publish_rate_ < 0.1)
                publish_rate_ = 5.0;

        }
    
        private:
            void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
            {
                ros::Time now = ros::Time::now();
                if ( (now-last_publish_time_).toSec() < 1.0/publish_rate_ )
                    return;
                last_publish_time_ = now;

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg, *cloud);

                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::VoxelGrid<pcl::PointXYZ> voxel;
                voxel.setInputCloud(cloud);
                voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
                voxel.filter(*filtered);

                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(*filtered, output);
                output.header = msg->header;
                pub_.publish(output);
            }

            ros::Subscriber sub_;
            ros::Publisher pub_;
            std::string input_topic_, output_topic_;
            double voxel_size_, publish_rate_;
            ros::Time last_publish_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_web_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CloudWebFilter filter(nh, pnh);

  ros::spin();
  return 0;
}