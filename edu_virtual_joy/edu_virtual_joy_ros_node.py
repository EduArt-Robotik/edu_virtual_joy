import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from edu_robot.srv import SetMode
from edu_robot.msg import Mode, SetLightingColor, RobotStatusReport

from threading import Thread
from functools import partial
from enum import IntEnum

class RangeSensor(IntEnum):
  FrontLeft = 0
  FrontRight = 1
  RearLeft = 2
  RearRight = 3
  NumSensors = 4

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
    self.callbacks_range_sensor = []
    self.callbacks_range_sensor.append([])
    self.callbacks_range_sensor.append([])
    self.callbacks_range_sensor.append([])
    self.callbacks_range_sensor.append([])

    self.callback_velocity = [] 

  def ros_thread_process(self):
    print('launching ros thread')
    rclpy.init(args=None)
    self.node = rclpy.create_node('edu_virtual_joy')
    self.sub_status = self.node.create_subscription(
      RobotStatusReport, '/eduard/blue/status_report', self.callback_status_report, QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2
      )
    )

    self.sub_range = []
    self.sub_range.append(self.node.create_subscription(
      Range, '/eduard/blue/range/front/left/range', lambda msg: EduVirtualJoyRosNode().callback_range_sensor(msg, RangeSensor.FrontLeft), QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2
      )      
    ))
    self.sub_range.append(self.node.create_subscription(
      Range, '/eduard/blue/range/front/right/range', lambda msg: EduVirtualJoyRosNode().callback_range_sensor(msg, RangeSensor.FrontRight), QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2
      )      
    ))
    self.sub_range.append(self.node.create_subscription(
      Range, '/eduard/blue/range/rear/left/range', lambda msg: EduVirtualJoyRosNode().callback_range_sensor(msg, RangeSensor.RearLeft), QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2
      )      
    ))
    self.sub_range.append(self.node.create_subscription(
      Range, '/eduard/blue/range/rear/right/range', lambda msg: EduVirtualJoyRosNode().callback_range_sensor(msg, RangeSensor.RearRight), QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2
      )
    ))

    self.pub_velocity = self.node.create_publisher(Twist, '/eduard/blue/cmd_vel', 1)
    self.pub_set_lighting = self.node.create_publisher(SetLightingColor, '/eduard/blue/set_lighting_color', 1)
    self.velocity_cmd = Twist()
    self.srv_set_mode = self.node.create_client(SetMode, '/eduard/blue/set_mode')
    self.timer_set_velocity = self.node.create_timer(0.1, self.set_velocity)

    print("start spinning node")
    rclpy.spin(self.node)
    self.node.destroy_node()
    rclpy.shutdown()

  def register_feedback_callback(self, callback):
    self.callback_feedback.append(callback)

  def remove_feedback_callback(self, callback):
    self.callback_feedback.remove(callback)

  def register_range_sensor_callback(self, callback, sensor_index : RangeSensor):
    self.callbacks_range_sensor[int(sensor_index)].append(callback)

  def remove_range_sensor_callback(self, callback, sensor_index : RangeSensor):
    self.callbacks_range_sensor[int(sensor_index)].remove(callback)

  def register_get_velocity_callback(self, callback):
    self.callback_velocity.append(callback)

  def remove_get_velocity_callback(self, callback):
    self.callback_velocity.remove(callback)

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

  def set_velocity(self):
    if len(self.callback_velocity) == 0:
      return

    # Add all velocities together. TODO: allow only one at same time
    velocity = [0.0, 0.0, 0.0]

    for callback in self.callback_velocity:
      value = callback()
      velocity[0] = velocity[0] + value[0]
      velocity[1] = velocity[1] + value[1]
      velocity[2] = velocity[2] + value[2]

    twist_msg = Twist()
    twist_msg.linear.x  = velocity[0]
    twist_msg.linear.y  = velocity[1]
    twist_msg.angular.z = velocity[2]

    self.pub_velocity.publish(twist_msg)

  def callback_status_report(self, msg):
    for callback in self.callback_feedback:
       callback(msg)

  def callback_range_sensor(self, msg, index : RangeSensor):
    for callback in self.callbacks_range_sensor[int(index)]:
      callback(msg, index)
