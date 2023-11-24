import flet as ft
import flet.canvas as cv

from functools import partial
from threading import Thread
import math

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
    for callback in self.callback_feedback:
       callback(msg)


class JoyStickWidget(ft.UserControl):
  def resizing(self, e):
    print('resizing')
    print(e)
    self.widget.shapes[0].x = e.width / 2
    self.widget.shapes[0].y = e.height / 2
    self.widget.shapes[1].x = e.width / 2
    self.widget.shapes[1].y = e.height / 2

    self.widget.width = e.width
    self.widget.height = e.width

    self.widget.update()

  def on_user_input(self, e):
    print('e.local_x = ', e.local_x)
    print('e.local_y = ', e.local_y)
    width = self.widget.width
    height = self.widget.height
    x = e.local_x - width / 2
    y = e.local_y - height / 2
    length = math.sqrt(x * x + y * y)
    print('length = ', length)

    if length > self.size / 2 - self.dot_radius:
      factor = (self.size / 2 - self.dot_radius) / float(length)
      x = float(x) * factor
      y = float(y) * factor

    print('x = ', x)
    print('y = ', y)
    self.x = x / float(self.size / 2)
    self.y = y / float(self.size / 2)

    self.widget.shapes[0].x = x + width / 2
    self.widget.shapes[0].y = y + height / 2
    self.widget.update()

  def on_user_input_release(self, e):
    self.widget.shapes[0].x = self.widget.width / 2
    self.widget.shapes[0].y = self.widget.height / 2
    self.widget.update()  

  def build(self):
    self.size = 300
    self.dot_radius = 30
    self.x = 0.0
    self.y = 0.0
    self.widget = cv.Canvas([
        cv.Circle(radius=self.dot_radius, paint=ft.Paint(color=ft.colors.RED)),
        cv.Circle(radius=self.size / 2, paint=ft.Paint(color=ft.colors.BLACK38))
      ],
      on_resize=self.resizing,
      width=self.size,  # size must be squared
      height=self.size,
      content=ft.GestureDetector(
        on_pan_update=self.on_user_input,
        on_pan_end=self.on_user_input_release,
        # on_tap_down=self.on_user_input,
        on_tap_up=self.on_user_input_release,
        on_exit=self.on_user_input_release,
        on_long_press_end=self.on_user_input_release,
        drag_interval=20
      )
    )

    return self.widget

  def value(self):
    return [self.x, self.y]

class EduVirtualJoyWebApp(ft.UserControl):
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

    # create joysticks
    joystick_left = JoyStickWidget()

    # register callback for receiving robot status reports
    EduVirtualJoyRosNode().register_feedback_callback(self.process_robot_feedback)

    return ft.Column([
      group_select_kinematic,
      group_switch_mode,
      joystick_left
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

  user_control = EduVirtualJoyWebApp()
  page.add(user_control)
  page.on_disconnect = lambda e: EduVirtualJoyRosNode().remove_feedback_callback(user_control.process_robot_feedback)

ft.app(main, view=ft.AppView.WEB_BROWSER, port=8888)