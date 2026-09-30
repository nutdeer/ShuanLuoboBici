# cam_front

这个节点只读取 USB 相机并发布 `/cam_front/image_raw`,不单独写视频文件。rosbag 会保存图像消息的 `header.stamp`,之后可以和里程计、IMU 按时间戳做近似同步。

```bash
cd /home/reallab_zwx/TEST_EPIC_C
catkin_make --pkg cam_front
source devel/setup.bash
roslaunch cam_front cam_front.launch
```

另一个终端录制:

```bash
rosbag record -O epic_flight.bag /cam_front/image_raw /odom /imu/data
```

## 参数说明

所有参数统一在 `config/cam_front.yaml` 中配置,launch 启动时自动加载;如果 YAML 里缺了某项,节点代码里还有一个兜底默认值(两者一致)。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device` | `/dev/video0` | V4L2 设备 |
| `capture_width` / `capture_height` | 640×480 | 相机采集分辨率(必须在相机支持列表内,见 `v4l2-ctl -d /dev/video0 --list-formats-ext`) |
| `fourcc` | `MJPG` | 采集像素格式,在设置分辨率之前生效 |
| `width` / `height` | 320×240 | 发布话题的 resize 输出尺寸 |
| `fps` | 15 | 帧率(相机采集与发布共用,想更省资源可降到 10) |
| `mono` | false | `true` 发布 `mono8` 黑白,`false` 发布 `bgr8` 彩色 |

`capture_width`/`capture_height` 设为 0 时,采集尺寸跟随输出尺寸。

节点做了懒发布:`/cam_front/image_raw` 没有任何订阅者时只抓帧丢弃(不做 MJPG 解码和图像处理),几乎不占 CPU;有订阅者(如 rosbag record)时自动恢复正常发布。

采集 1080p、发布 640×480 彩色:把 YAML 里改成

```yaml
capture_width: 1920
capture_height: 1080
width: 640
height: 480
```

启动时节点会打印实际生效的采集和输出配置。

`frame_id` 是图像的 ROS 坐标系标记,不是叠加在画面上的文字。
