  #include <ros/ros.h>
  #include <nav_msgs/Odometry.h>

  class OdomWebFilter
{
    public:
        OdomWebFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            // 构造函数定义
            pnh.param<std::string>("input_topic", input_topic_, "/Odometry_high_freq");
            pnh.param<std::string>("output_topic", output_topic_, "/Odometry_web");

            pnh.param<double>("publish_rate", publish_rate_, 10.0);  // Hz

            pub_ = nh.advertise<nav_msgs::Odometry>(output_topic_, 1);
            sub_ = nh.subscribe(input_topic_, 1, &OdomWebFilter::odomCallback, this);

            last_publish_time_ = ros::Time(0);

            ROS_INFO("[odom_web_filter] %s -> %s, rate=%.1f Hz",
                    input_topic_.c_str(), output_topic_.c_str(), publish_rate_);
            
            if(publish_rate_ < 0.1)
                publish_rate_ = 10.0;
        }
    
        private:
            void odomCallback(const nav_msgs::OdometryConstPtr &msg)
            {
                ros::Time now = ros::Time::now();
                if ( (now-last_publish_time_).toSec() < 1.0/publish_rate_ )
                    return;

                last_publish_time_ = now;
                pub_.publish(msg);
            }

            ros::Subscriber sub_;
            ros::Publisher pub_;
            std::string input_topic_, output_topic_;
            double publish_rate_;
            ros::Time last_publish_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_web_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  OdomWebFilter filter(nh, pnh);

  ros::spin();
  return 0;
}