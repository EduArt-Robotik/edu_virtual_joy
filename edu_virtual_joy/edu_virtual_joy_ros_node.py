import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from edu_robot.srv import SetMode
from edu_robot.msg import Mode, SetLightingColor, RobotStatusReport

from threading import Thread

class EduVirtualJoyRosNode():
  _instance = None

  def __new__(cls):
    if cls._instance is None:
        print('creating ros instance')
        cls._instance = super(EduVirtualJoyRosNode, cls).__new__(cls)
        cls._instance.initialize()

    return cls._instance

  def initialize(self):
    self.srv_set_mode = None
    self.pub_set_lighting = None
    self.pub_velocity = None

    self.ros_thread = Thread(target=self.ros_thread_process)
    self.ros_thread.start()
    self.callback_feedback = []

  def ros_thread_process(self):
    print('launching ros thread')
    rclpy.init(args=None)
    self.node = rclpy.create_node('edu_virtual_joy')
    self.sub_status = self.node.create_subscription(
      RobotStatusReport, '/eduard/blue/status_report', self.callback_status_report, QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100
      )
    )
    self.pub_velocity = self.node.create_publisher(Twist, '/eduard/blue/cmd_vel', 1)
    self.pub_set_lighting = self.node.create_publisher(SetLightingColor, '/eduard/blue/set_lighting_color', 1)
    self.velocity_cmd = Twist()
    self.srv_set_mode = self.node.create_client(SetMode, '/eduard/blue/set_mode')
    print("start spinning node")
    rclpy.spin(self.node)
    self.node.destroy_node()
    rclpy.shutdown()

  def register_feedback_callback(self, callback):
    self.callback_feedback.append(callback)

  def remove_feedback_callback(self, callback):
    self.callback_feedback.remove(callback)

  def set_mode_robot(self, mode) -> None:
    print('try to set robot mode')
    if self.srv_set_mode is None:
       return

    set_mode_request = SetMode.Request()
    set_mode_request.mode.mode = mode

    if (self.srv_set_mode.service_is_ready() is False):
        print('Service set mode not ready --> cancel operation')
        return

    future = self.srv_set_mode.call_async(set_mode_request)

  def set_drive_kinematic(self, kinematic) -> None:
    set_mode_request = SetMode.Request()
    set_mode_request.mode.drive_kinematic = kinematic

    if (self.srv_set_mode.service_is_ready() is False):
        print('Service set mode not ready --> cancel operation')
        return

    future = self.srv_set_mode.call_async(set_mode_request)

  def callback_status_report(self, msg):
    for callback in self.callback_feedback:
       callback(msg)