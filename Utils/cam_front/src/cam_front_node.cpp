#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_front");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport image_transport(nh);
  image_transport::Publisher image_pub = image_transport.advertise("image_raw", 1);

  std::string device, frame_id;
  int width, height;
  double fps;
  nh.param("device", device, std::string("/dev/video0"));
  nh.param("frame_id", frame_id, std::string("camera_front"));
  nh.param("width", width, 320);
  nh.param("height", height, 240);
  nh.param("fps", fps, 30.0);

  cv::VideoCapture camera(device, cv::CAP_V4L2);
  if (!camera.isOpened()) {
    ROS_ERROR_STREAM("Cannot open camera: " << device);
    return 1;
  }
  camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
  camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  camera.set(cv::CAP_PROP_FPS, fps);

  ros::Rate rate(fps);
  cv::Mat frame;
  while (ros::ok()) {
    if (!camera.read(frame) || frame.empty()) {
      ROS_ERROR("Camera read failed");
      break;
    }

    const ros::Time stamp = ros::Time::now();
    cv::Mat gray;
    if (frame.channels() == 1) {
      gray = frame;
    } else if (frame.channels() == 3) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
      ROS_ERROR_STREAM("Unsupported image channels: " << frame.channels());
      break;
    }
    cv::Mat small_frame;
    cv::resize(gray, small_frame, cv::Size(width, height));

    // The timestamp is stored in the ROS message and can be used for sync.
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    image_pub.publish(cv_bridge::CvImage(header, "mono8", small_frame).toImageMsg());

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
