import flet as ft

from functools import partial
from threading import Thread

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from edu_robot.srv import SetMode
from edu_robot.msg import Mode, SetLightingColor, RobotStatusReport

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
    print('received robot status report')
    for callback in self.callback_feedback:
       callback(msg)


class EduVirtualJoyWebApp(ft.UserControl):
  def __del__(self):
    print("destructor called")
    EduVirtualJoyRosNode().remove_feedback_callback(self.process_robot_feedback)

  def will_unmount(self):
    print("will unmount called")
    EduVirtualJoyRosNode().remove_feedback_callback(self.process_robot_feedback)

  def build(self):
    # create input control group switch robot mode
    self.remote_button = ft.ElevatedButton(
      "Remote", on_click=lambda e : EduVirtualJoyRosNode().set_mode_robot(Mode.REMOTE_CONTROLLED), disabled=True)
    self.autonomous_button = ft.ElevatedButton(
      "Autonomous", on_click=lambda e : EduVirtualJoyRosNode().set_mode_robot(Mode.AUTONOMOUS), disabled=True)
    self.disable_button = ft.ElevatedButton(
      "Disable", on_click=lambda e : EduVirtualJoyRosNode().set_mode_robot(Mode.INACTIVE), disabled=True)

    group_switch_mode = ft.Row([
        self.remote_button,
        self.autonomous_button,
        self.disable_button
      ],
      alignment=ft.MainAxisAlignment.CENTER,
    )

    # create input control group select robot drive kinematic
    self.select_skip_drive = ft.ElevatedButton(
      "Skid", on_click=lambda e : EduVirtualJoyRosNode().set_drive_kinematic(Mode.SKID_DRIVE), disabled=True)
    self.select_mecanum = ft.ElevatedButton(
      "Mecanum", on_click=lambda e : EduVirtualJoyRosNode().set_drive_kinematic(Mode.MECANUM_DRIVE), disabled=True)

    group_select_kinematic = ft.Row([
        self.select_skip_drive,
        self.select_mecanum
      ],
      alignment=ft.MainAxisAlignment.CENTER
    )
    
    # register callback for receiving robot status reports
    EduVirtualJoyRosNode().register_feedback_callback(self.process_robot_feedback)

    return ft.Column([
      group_select_kinematic,
      group_switch_mode
    ])

  def process_robot_feedback(self, status_report : RobotStatusReport):
    print('process robot status report')
    print(status_report)

    # process robot mode
    if status_report.robot_state.mode.mode is Mode.REMOTE_CONTROLLED:
      # robot is in mode REMOTE CONTROLLED
      ## set color according mode
      self.remote_button.color = ft.colors.GREEN
      self.autonomous_button.color = ft.colors.BLACK
      self.disable_button.color = ft.colors.BLACK

      ## set disabled state according mode
      self.remote_button.disabled = True
      self.autonomous_button.disabled = False
      self.disable_button.disabled = False

    elif status_report.robot_state.mode.mode is Mode.AUTONOMOUS:
      # robot is in mode AUTONOMOUS
      ## set color according mode
      self.remote_button.color = ft.colors.BLACK
      self.autonomous_button.color = ft.colors.GREEN
      self.disable_button.color = ft.colors.BLACK

      ## set disabled state according mode
      self.remote_button.disabled = False
      self.autonomous_button.disabled = True
      self.disable_button.disabled = False    

    elif status_report.robot_state.mode.mode is Mode.CHARGING:
      # robot is in mode CHARGING
      ## set color according mode
      self.remote_button.color = ft.colors.BLACK
      self.autonomous_button.color = ft.colors.BLACK
      self.disable_button.color = ft.colors.BLACK

      ## set disabled state according mode
      self.remote_button.disabled = True
      self.autonomous_button.disabled = True
      self.disable_button.disabled = True

    else:
      # robot is in mode REMOTE CONTROLLED
      ## set color according mode
      self.remote_button.color = ft.colors.BLACK
      self.autonomous_button.color = ft.colors.BLACK
      self.disable_button.color = ft.colors.RED

      ## set disabled state according mode
      self.remote_button.disabled = False
      self.autonomous_button.disabled = False
      self.disable_button.disabled = True
    
    self.remote_button.update()
    self.autonomous_button.update()
    self.disable_button.update()

    # process drive kinematic
    if status_report.robot_state.mode.drive_kinematic is Mode.SKID_DRIVE:
      self.select_skip_drive.disabled = True
      self.select_mecanum.disabled = False
    elif status_report.robot_state.mode.drive_kinematic is Mode.MECANUM_DRIVE:
      self.select_skip_drive.disabled = False
      self.select_mecanum.disabled = True

    self.select_skip_drive.update()
    self.select_mecanum.update()


def main(page: ft.Page):
  page.title = "EduArt Virtual Joy"
  page.vertical_alignment = ft.MainAxisAlignment.CENTER
  page.update()

  page.add(EduVirtualJoyWebApp())

ft.app(main, view=ft.AppView.WEB_BROWSER, port=8888)