import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import TakeoffLand


class RvizDragTakeoff:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_rviz")
        self.takeoff_land_topic = rospy.get_param(
            "~takeoff_land_topic", "/px4ctrl/takeoff_land")
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 0.5))
        self.repeat = int(rospy.get_param("~repeat", 20))

        self.pub = rospy.Publisher(self.takeoff_land_topic, TakeoffLand, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=1)

        rospy.loginfo("[rviz_drag_takeoff] %s -> %s, takeoff_height=%.2fm",
                      self.goal_topic, self.takeoff_land_topic, self.takeoff_height)

    def goal_cb(self, goal):
        dz = goal.pose.position.z
        if dz > 0.0:
            self.send(TakeoffLand.TAKEOFF, self.takeoff_height)
            rospy.logwarn("[rviz_drag_takeoff] TAKEOFF -> %.2fm", self.takeoff_height)
        elif dz < 0.0:
            self.send(TakeoffLand.LAND, 0.0)
            rospy.logwarn("[rviz_drag_takeoff] LAND")

    def send(self, cmd, height):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = cmd
        msg.takeoff_hight = height
        rate = rospy.Rate(100)
        for _ in range(max(1, self.repeat)):
            if rospy.is_shutdown():
                break
            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rviz_drag_takeoff")
    RvizDragTakeoff()
    rospy.spin()
