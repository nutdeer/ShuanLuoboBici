import copy
import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand

spec=importlib.util.spec_from_file_location('bridge',Path(__file__).resolve().parents[1]/'scripts/sim_takeoff_bridge.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(copy.deepcopy(msg))
    def get_num_connections(self):return 1

class BridgeTest(unittest.TestCase):
    def test_refuses_non_simulation_startup(self):
        with patch.object(rospy,'get_param',return_value=False):
            with self.assertRaises(RuntimeError):module.Bridge()
        with patch.object(rospy,'get_param',return_value=True),patch.object(rospy,'resolve_name',return_value='/planning/pos_cmd'):
            with self.assertRaises(RuntimeError):module.Bridge()

    def test_takeoff_deduplication_stabilization_and_handoff(self):
        b=module.Bridge.__new__(module.Bridge)
        b.lock=threading.RLock();b.mode='WAIT_TAKEOFF';b.odom=Odometry()
        b.odom.pose.pose.orientation.w=1
        b.duration=3.;b.stable_duration=1.;b.height_tolerance=.05;b.velocity_tolerance=.1
        b.stable_since=None;b.started=None;b.output=Publisher();b.status=Publisher()
        b.goal_output=Publisher();b.takeoff_requested=False;b.exploration_requested=False
        goal=PoseStamped();goal.pose.position.z=1
        msg=TakeoffLand();msg.takeoff_land_cmd=TakeoffLand.TAKEOFF;msg.takeoff_hight=.5
        planned=PositionCommand();planned.position.x=5
        with patch.object(rospy.Time,'now') as now,patch.object(rospy,'loginfo'),patch.object(rospy,'logwarn_throttle'):
            now.return_value=rospy.Time(10)
            b.on_plan(planned);self.assertEqual(len(b.output.messages),0)
            b.on_goal(goal);b.on_goal(goal)
            self.assertEqual(len(b.goal_output.messages),1)
            b.on_takeoff(msg);self.assertEqual(b.mode,'TAKING_OFF')
            b.on_goal(goal);self.assertEqual(len(b.goal_output.messages),1)
            initial=b.started
            for i in range(20):
                now.return_value=rospy.Time(10+i*.01);b.on_takeoff(msg)
            self.assertEqual(b.started,initial)
            now.return_value=rospy.Time(11.5);b.tick(None)
            self.assertAlmostEqual(b.output.messages[-1].position.z,.25)
            self.assertGreater(b.output.messages[-1].velocity.z,0)
            b.on_plan(planned);self.assertNotEqual(b.output.messages[-1].position.x,5)
            now.return_value=rospy.Time(13);b.tick(None)
            self.assertAlmostEqual(b.output.messages[-1].position.z,.5)
            self.assertAlmostEqual(b.output.messages[-1].velocity.z,0)
            self.assertEqual(b.mode,'TAKING_OFF')
            b.odom.pose.pose.position.z=.5
            now.return_value=rospy.Time(14);b.tick(None)
            now.return_value=rospy.Time(15);b.tick(None)
            self.assertEqual(b.mode,'HOVER')
            count=len(b.output.messages);b.on_plan(planned)
            self.assertEqual(len(b.output.messages),count)
            b.on_goal(goal);self.assertEqual(len(b.goal_output.messages),2)
            b.on_plan(planned);self.assertEqual(b.mode,'EXPLORING')
            self.assertEqual(b.output.messages[-1].position.x,5)
            count=len(b.output.messages);b.tick(None)
            self.assertEqual(len(b.output.messages),count)
            b.on_takeoff(msg);self.assertEqual(b.mode,'EXPLORING')

if __name__=='__main__':unittest.main()
