import flet as ft

import math

from edu_virtual_joy_ros_node import EduVirtualJoyRosNode, RangeSensor
from joy_stick_control import JoyStickControl
from eduard_status_display import EduardStatusReportDisplay

from edu_robot.msg import Mode, RobotStatusReport

class EduVirtualJoyWebApp(ft.UserControl):
  def initialize(self):
    # register callbacks for receiving robot status reports, sensor data
    EduVirtualJoyRosNode().register_feedback_callback(self.process_robot_feedback)
    EduVirtualJoyRosNode().register_range_sensor_callback(
      self.eduard_status_display.set_distance, RangeSensor.FrontLeft)
    EduVirtualJoyRosNode().register_range_sensor_callback(
      self.eduard_status_display.set_distance, RangeSensor.FrontRight)
    EduVirtualJoyRosNode().register_range_sensor_callback(
      self.eduard_status_display.set_distance, RangeSensor.RearLeft)
    EduVirtualJoyRosNode().register_range_sensor_callback(
      self.eduard_status_display.set_distance, RangeSensor.RearRight)
    EduVirtualJoyRosNode().register_get_velocity_callback(self.get_velocity)

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
    self.joystick_left  = JoyStickControl()
    self.joystick_right = JoyStickControl()

    # create Eduard status display
    self.eduard_status_display = EduardStatusReportDisplay()

    group_joy_and_status = ft.Row([
        self.joystick_left,
        self.eduard_status_display,
        self.joystick_right
      ],
      alignment=ft.MainAxisAlignment.CENTER
    )

    return ft.Column([
      group_joy_and_status,
      group_select_kinematic,
      group_switch_mode,
    ])

  def get_velocity(self):
    return [ self.joystick_left.get_relative_position()[1] * -1.5,
             self.joystick_left.get_relative_position()[0] * -1.5,
             self.joystick_right.get_relative_position()[0] * -math.pi ]

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
      self.eduard_status_display.show_offroad()
    elif status_report.robot_state.mode.drive_kinematic is Mode.MECANUM_DRIVE:
      self.select_skip_drive.disabled = False
      self.select_mecanum.disabled = True
      self.eduard_status_display.show_mecanum()

    self.select_skip_drive.update()
    self.select_mecanum.update()


def main(page: ft.Page):
  page.title = "EduArt Virtual Joy"
  page.vertical_alignment = ft.MainAxisAlignment.CENTER
  page.update()

  user_control = EduVirtualJoyWebApp()
  page.add(user_control)
  page.on_disconnect = lambda e: (
    EduVirtualJoyRosNode().remove_feedback_callback(user_control.process_robot_feedback),
    EduVirtualJoyRosNode().remove_range_sensor_callback(
      user_control.eduard_status_display.set_distance, RangeSensor.FrontLeft),
    EduVirtualJoyRosNode().remove_range_sensor_callback(
      user_control.eduard_status_display.set_distance, RangeSensor.FrontRight),
    EduVirtualJoyRosNode().remove_range_sensor_callback(
      user_control.eduard_status_display.set_distance, RangeSensor.RearLeft),
    EduVirtualJoyRosNode().remove_range_sensor_callback(
      user_control.eduard_status_display.set_distance, RangeSensor.RearRight),
    EduVirtualJoyRosNode().remove_get_velocity_callback(
      user_control.get_velocity)
  )
  user_control.initialize()

ft.app(main, view=ft.AppView.WEB_BROWSER, port=8888, assets_dir="assets")