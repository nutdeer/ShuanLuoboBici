#include <pointcloudmap_load/globalmap_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <Eigen/Eigen>

using namespace std;
vector<double> _state;
random_device rd;
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_w,rand_h;

double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _sense_rate, _init_x, _init_y;
double _resolution = 0.25;
double _min_dist;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
pcl::PointCloud<pcl::PointXYZI> clicked_cloud_;
ros::Publisher click_map_pub_;
sensor_msgs::PointCloud2 localMap_pcd;
bool _has_odom = false;

GlobalmapServer globalmap_server;

void reciveOdometryCallbck(const nav_msgs::Odometry odom) {
  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

void clickCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
 
  /* ---------- only publish obstacle in given position ---------- */
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double w = rand_w(eng);
  double h;
  int widNum;
  int heiNum;
  pcl::PointXYZ pt_random;
  /* ---------- only publish obstacle around current position ---------- */

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  // double odom_x = _state[0];
  // double odom_y = _state[1];
  // double odom_z = _state[2];
  // double vel_x = _state[3];
  // double vel_y = _state[4];
  double yaw = 0; 

  double obs_distance = 0;
  double around_w = 0.5;
  pcl::PointXYZI around_random1, around_random2;

  // odom_x1 = floor(odom_x / _resolution) * _resolution + _resolution / 2.0;
  // odom_y1 = floor(odom_y / _resolution) * _resolution + _resolution / 2.0;
  widNum = ceil(around_w / _resolution);

  std::cout<<"width : " << widNum <<std::endl;
  // 8.18  -7.1  7.00  2 (floor-2) 
  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      for (int t = 0 ; t < 4; t++) {
        around_random1.x = globalmap_server.odom_x1 + obs_distance * cos(yaw) + (r + 0.5) * _resolution + 1e-2;
        around_random1.y = globalmap_server.odom_y1 + obs_distance * sin(yaw) + (s + 0.5) * _resolution + 1e-2;
        around_random1.z = globalmap_server.odom_z1 + _resolution*t ;
        clicked_cloud_.points.push_back(around_random1);
      }
    }
  // 8.18  -3.62  10.00  (floor-3)
  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      for (int t = 0  ; t < 4; t++) {
        around_random2.x = globalmap_server.odom_x2 + obs_distance * cos(yaw) + (r + 0.5) * _resolution + 1e-2;
        around_random2.y = globalmap_server.odom_y2 + obs_distance * sin(yaw) + (s + 0.5) * _resolution + 1e-2;
        around_random2.z = globalmap_server.odom_z2 + _resolution*t ;
        clicked_cloud_.points.push_back(around_random2);
      }
    }
  globalmap_server.loop_pub(clicked_cloud_);
  std::cout<<"road obstacle map pub ! "<<std::endl;

  return;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv,"globalmap_server");
    ros::NodeHandle nh_("~");
    globalmap_server.init(nh_);
    std::cerr << globalmap_server.odom_x1 << "\t" << globalmap_server.odom_y1 << "\t" << globalmap_server.odom_x1 << "\t" << std::endl;
    ROS_INFO("\033[1;32m---->\033[0m [%s] Started.", ros::this_node::getName().c_str() );

    ros::Subscriber click_sub = nh_.subscribe("/initialpose", 10, clickCallback);
    ros::Subscriber odom_sub = nh_.subscribe("/drone_0_visual_slam/odom", 10, reciveOdometryCallbck);

    ros::spin();
    return 0;
}