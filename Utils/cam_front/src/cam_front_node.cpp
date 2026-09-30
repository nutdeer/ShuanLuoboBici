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

  std::string device, frame_id, fourcc;
  int width, height, capture_width, capture_height;
  double fps;
  bool mono;
  nh.param("device", device, std::string("/dev/video0"));
  nh.param("frame_id", frame_id, std::string("camera_front"));
  nh.param("width", width, 320);
  nh.param("height", height, 240);
  nh.param("fps", fps, 30.0);
  nh.param("mono", mono, false);
  nh.param("capture_width", capture_width, 0);
  nh.param("capture_height", capture_height, 0);
  nh.param("fourcc", fourcc, std::string("MJPG"));
  if (capture_width <= 0) capture_width = width;
  if (capture_height <= 0) capture_height = height;

  cv::VideoCapture camera(device, cv::CAP_V4L2);
  if (!camera.isOpened()) {
    ROS_ERROR_STREAM("Cannot open camera: " << device);
    return 1;
  }
  // FOURCC must be set before width/height, otherwise the driver keeps its
  // default format and may reject the requested resolution.
  if (fourcc.size() == 4) {
    camera.set(cv::CAP_PROP_FOURCC,
               cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
  }
  camera.set(cv::CAP_PROP_FRAME_WIDTH, capture_width);
  camera.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height);
  camera.set(cv::CAP_PROP_FPS, fps);
  ROS_INFO_STREAM("Camera capture: " << capture_width << "x" << capture_height << " @ " << fps
                                    << " fps, fourcc=" << fourcc << ", output: " << width << "x"
                                    << height << (mono ? " mono8" : " bgr8"));

  ros::Rate rate(fps);
  cv::Mat frame;
  while (ros::ok()) {
    if (image_pub.getNumSubscribers() == 0) {
      // No subscribers: grab and discard to keep the driver buffer fresh,
      // skipping MJPG decode, color conversion, resize and serialization.
      camera.grab();
      ros::spinOnce();
      rate.sleep();
      continue;
    }
    if (!camera.read(frame) || frame.empty()) {
      ROS_ERROR("Camera read failed");
      break;
    }

    const ros::Time stamp = ros::Time::now();
    cv::Mat out_frame;
    if (mono) {
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
      cv::resize(gray, out_frame, cv::Size(width, height));
    } else {
      cv::Mat bgr;
      if (frame.channels() == 3) {
        bgr = frame;
      } else if (frame.channels() == 4) {
        cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
      } else if (frame.channels() == 1) {
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
      } else {
        ROS_ERROR_STREAM("Unsupported image channels: " << frame.channels());
        break;
      }
      cv::resize(bgr, out_frame, cv::Size(width, height));
    }

    // The timestamp is stored in the ROS message and can be used for sync.
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    image_pub.publish(
        cv_bridge::CvImage(header, mono ? "mono8" : "bgr8", out_frame).toImageMsg());

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
