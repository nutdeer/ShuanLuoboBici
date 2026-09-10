# cam_front

这个节点只读取 USB 相机并发布 `/cam_front/image_raw`，不单独写视频文件。rosbag 会保存图像消息的 `header.stamp`，之后可以和里程计、IMU 按时间戳做近似同步。

```bash
cd /home/reallab_zwx/TEST_EPIC_C
catkin_make --pkg cam_front
source devel/setup.bash
roslaunch cam_front cam_front.launch
```

另一个终端录制：

```bash
rosbag record -O epic_flight.bag /cam_front/image_raw /odom /imu/data
```

默认相机是 `/dev/video0`，发布黑白 `mono8` 图像，输出分辨率 320×240，帧率 30：

```bash
roslaunch cam_front cam_front.launch device:=/dev/video0 width:=640 height:=480 fps:=30
```

`frame_id` 是图像的 ROS 坐标系标记，不是叠加在画面上的文字。
