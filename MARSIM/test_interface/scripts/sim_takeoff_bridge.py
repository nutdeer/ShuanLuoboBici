#!/usr/bin/env python3
"""Simulation-only arbiter between takeoff and planner position commands."""
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import String


def climb_profile(elapsed, duration):
    u=max(0.0,min(1.0,elapsed/duration))
    return (10*u**3-15*u**4+6*u**5,
            (30*u**2-60*u**3+30*u**4)/duration,
            (60*u-180*u**2+120*u**3)/duration**2)


class Bridge:
    def __init__(self):
        if not rospy.get_param('~simulation_only',False):
            raise RuntimeError('This adapter requires the simulation-only launch')
        for topic in ('~controller_cmd','~takeoff_land','~goal_out'):
            if not rospy.resolve_name(topic).startswith('/sim/'):
                raise RuntimeError('Simulation adapter requires /sim/ topics: '+topic)
        self.lock=threading.RLock()
        self.mode='WAIT_TAKEOFF'
        self.odom=None
        self.duration=rospy.get_param('~climb_duration',3.0)
        self.stable_duration=rospy.get_param('~stable_duration',1.0)
        self.height_tolerance=rospy.get_param('~height_tolerance',0.05)
        self.velocity_tolerance=rospy.get_param('~velocity_tolerance',0.10)
        self.stable_since=None
        self.started=None
        self.takeoff_requested=False
        self.exploration_requested=False
        if not math.isfinite(self.duration) or self.duration<=0:
            raise ValueError('climb_duration must be positive and finite')
        self.output=rospy.Publisher('~controller_cmd',PositionCommand,queue_size=10)
        self.status=rospy.Publisher('~state',String,queue_size=1,latch=True)
        self.goal_output=rospy.Publisher('~goal_out',PoseStamped,queue_size=1)
        self.status.publish(String(data=self.mode))
        rospy.Subscriber('~odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('~takeoff_land',TakeoffLand,self.on_takeoff,queue_size=10)
        rospy.Subscriber('~planner_cmd',PositionCommand,self.on_plan,queue_size=1)
        rospy.Subscriber('~goal_in',PoseStamped,self.on_goal,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(0.02),self.tick)

    def change_mode(self,mode):
        self.mode=mode
        self.status.publish(String(data=mode))
        rospy.loginfo('Simulation takeoff state: %s',mode)

    def on_odom(self,msg):
        with self.lock:self.odom=msg

    def on_goal(self,msg):
        with self.lock:
            if msg.pose.position.z<=0:
                rospy.logwarn_throttle(2.0,'Simulation adapter supports upward takeoff/exploration triggers only')
                return
            if self.odom is None or self.goal_output.get_num_connections()==0:
                rospy.logwarn_throttle(2.0,'Wait for simulation odometry and planner before triggering')
                return
            if self.mode=='WAIT_TAKEOFF' and not self.takeoff_requested:
                self.takeoff_requested=True
                self.goal_output.publish(msg)
            elif self.mode=='HOVER' and not self.exploration_requested:
                self.exploration_requested=True
                self.goal_output.publish(msg)
            elif self.mode=='TAKING_OFF':
                rospy.logwarn_throttle(2.0,'Takeoff not stable yet; trigger exploration after HOVER')

    def on_takeoff(self,msg):
        with self.lock:
            # The planner publishes the same TAKEOFF command 20 times.
            if msg.takeoff_land_cmd!=TakeoffLand.TAKEOFF or self.mode!='WAIT_TAKEOFF':return
            if self.odom is None:
                rospy.logwarn_throttle(1.0,'Simulation takeoff waiting for odometry')
                return
            h=float(msg.takeoff_hight)
            if not math.isfinite(h) or h<=0:
                rospy.logerr('Invalid relative takeoff height: %s',h)
                return
            p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
            self.origin=(p.x,p.y,p.z)
            self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.height=h
            self.started=rospy.Time.now()
            self.change_mode('TAKING_OFF')

    def on_plan(self,msg):
        with self.lock:
            if self.mode not in ('HOVER','EXPLORING') or not self.exploration_requested:
                rospy.logwarn_throttle(2.0,'Ignoring planner command before takeoff is stable; wait for HOVER before the second trigger')
                return
            if self.mode=='HOVER':self.change_mode('EXPLORING')
            self.output.publish(msg)

    def tick(self,event):
        with self.lock:
            if self.mode not in ('TAKING_OFF','HOVER'):return
            elapsed=(rospy.Time.now()-self.started).to_sec()
            s,ds,dds=climb_profile(elapsed,self.duration)
            cmd=PositionCommand()
            cmd.header.stamp=rospy.Time.now();cmd.header.frame_id='world'
            cmd.position.x,cmd.position.y=self.origin[:2]
            cmd.position.z=self.origin[2]+self.height*s
            cmd.velocity.z=self.height*ds
            cmd.acceleration.z=self.height*dds
            cmd.yaw=self.yaw
            cmd.trajectory_flag=PositionCommand.TRAJECTORY_STATUS_READY
            self.output.publish(cmd)
            if self.mode=='TAKING_OFF' and elapsed>=self.duration and self.odom is not None:
                close=abs(self.odom.pose.pose.position.z-cmd.position.z)<self.height_tolerance
                slow=abs(self.odom.twist.twist.linear.z)<self.velocity_tolerance
                if close and slow:
                    if self.stable_since is None:self.stable_since=rospy.Time.now()
                    if (rospy.Time.now()-self.stable_since).to_sec()>=self.stable_duration:
                        self.change_mode('HOVER')
                else:self.stable_since=None


if __name__=='__main__':
    rospy.init_node('sim_takeoff_bridge')
    bridge=Bridge()
    rospy.spin()
