#!/usr/bin/env python3
"""Keyboard controller for the project's absolute-pose Twist bridge.

This is intentionally not compatible with teleop_twist_keyboard: the existing
bridge interprets Twist.linear as target world pose (x, y, yaw), not velocity.
"""

import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf.transformations import euler_from_quaternion


HELP = """
Manual table scan controls
--------------------------
  w / s       forward / backward one step
  a / d       strafe left / right one step
  q / e       rotate left / right one step
  x           stop at the measured pose
  SPACE       enable TSDF capture (press only after settling)
  c           disable capture before moving
  p           save PLY + PCD
  r           reset TSDF and table ROI
  h           show this help
  Ctrl-C      quit
"""


class KeyboardScanTeleop:
    def __init__(self):
        rospy.init_node("mm_keyboard_scan_teleop")
        self.command_topic = rospy.get_param(
            "~command_topic", "/mm_controller_node/car_cmd")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/robot1/robot1_agv_base_link_position")
        self.capture_topic = rospy.get_param(
            "~capture_topic", "/table_fine_mapping/capture")
        self.linear_step = float(rospy.get_param("~linear_step", 0.12))
        self.yaw_step = math.radians(float(rospy.get_param("~yaw_step_deg", 7.5)))
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))

        self.command_pub = rospy.Publisher(
            self.command_topic, Twist, queue_size=1)
        self.capture_pub = rospy.Publisher(
            self.capture_topic, Bool, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=1)

        self.save_service = rospy.get_param(
            "~save_service", "/table_fine_mapper/save")
        self.reset_service = rospy.get_param(
            "~reset_service", "/table_fine_mapper/reset")

        self.measured_pose = None
        self.target_pose = None
        self.terminal_settings = termios.tcgetattr(sys.stdin)
        self.capture_pub.publish(Bool(data=False))

    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.measured_pose = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw]
        if self.target_pose is None:
            self.target_pose = list(self.measured_pose)
            rospy.loginfo("[keyboard_scan] initialized at x=%.3f y=%.3f yaw=%.1f deg",
                          self.target_pose[0], self.target_pose[1],
                          math.degrees(self.target_pose[2]))

    @staticmethod
    def _read_key(timeout):
        tty.setraw(sys.stdin.fileno())
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if readable else ""

    def _capture(self, enabled):
        self.capture_pub.publish(Bool(data=enabled))
        rospy.loginfo("[keyboard_scan] TSDF capture %s", "ON" if enabled else "OFF")

    @staticmethod
    def _call_trigger(service_name):
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
            response = rospy.ServiceProxy(service_name, Trigger)()
            rospy.loginfo("[keyboard_scan] %s", response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("[keyboard_scan] service %s failed: %s", service_name, exc)

    def _move_body(self, forward, left):
        yaw = self.target_pose[2]
        self.target_pose[0] += math.cos(yaw) * forward - math.sin(yaw) * left
        self.target_pose[1] += math.sin(yaw) * forward + math.cos(yaw) * left

    def _handle_key(self, key):
        if not key:
            return
        if key in ("w", "s", "a", "d", "q", "e", "x"):
            self._capture(False)
        if key == "w":
            self._move_body(self.linear_step, 0.0)
        elif key == "s":
            self._move_body(-self.linear_step, 0.0)
        elif key == "a":
            self._move_body(0.0, self.linear_step)
        elif key == "d":
            self._move_body(0.0, -self.linear_step)
        elif key == "q":
            self.target_pose[2] = self._normalize(self.target_pose[2] + self.yaw_step)
        elif key == "e":
            self.target_pose[2] = self._normalize(self.target_pose[2] - self.yaw_step)
        elif key == "x" and self.measured_pose is not None:
            self.target_pose = list(self.measured_pose)
        elif key == " ":
            self._capture(True)
        elif key == "c":
            self._capture(False)
        elif key == "p":
            self._call_trigger(self.save_service)
        elif key == "r":
            self._capture(False)
            self._call_trigger(self.reset_service)
        elif key == "h":
            print(HELP)
        elif key == "\x03":
            raise KeyboardInterrupt

    def _publish_command(self):
        if self.target_pose is None:
            return
        msg = Twist()
        msg.linear.x = self.target_pose[0]
        msg.linear.y = self.target_pose[1]
        msg.linear.z = self.target_pose[2]
        self.command_pub.publish(msg)

    def run(self):
        print(HELP)
        rate = rospy.Rate(self.publish_rate)
        try:
            while not rospy.is_shutdown():
                key = self._read_key(0.02)
                if self.target_pose is not None:
                    self._handle_key(key)
                    self._publish_command()
                rate.sleep()
        finally:
            self._capture(False)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)


if __name__ == "__main__":
    try:
        KeyboardScanTeleop().run()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
