import flet as ft
import flet.canvas as cv

import math

from edu_virtual_joy_ros_node import RangeSensor
from images import get_mecanum_image, get_offroad_image

class RangeSensorDisplay(ft.UserControl):
  def __init__(self, top_down : bool):
    super().__init__()
    self.top_down = top_down
    self.text_size = 20

  def set_distance(self, distance : float):
    if distance != distance:
      # distance is nan
      distance = 0.0

    self.distance = min(distance, self.max_distance)
    distance_scale = self.distance / self.max_distance
    path_height = self.height - self.text_size

    # recalculate path elements based on new distance value
    self.canvas.shapes[0].elements[0] = cv.Path.MoveTo(self.width / 2, 0)
    self.canvas.shapes[0].elements[1] = cv.Path.LineTo(
      math.sin(-self.fov / 2) * path_height * distance_scale + self.width / 2,
      math.cos(-self.fov / 2) * path_height * distance_scale
    )
    self.canvas.shapes[0].elements[2] = cv.Path.QuadraticTo(
      self.width / 2,
      path_height * distance_scale,
      math.sin(self.fov / 2) * path_height * distance_scale + self.width / 2,
      math.cos(self.fov / 2) * path_height * distance_scale
    )
    self.canvas.shapes[0].elements[3] = cv.Path.LineTo(self.width / 2, 0)
    self.canvas.shapes[1] = cv.Text(
      self.width / 2,
      (path_height + self.text_size / 2) * distance_scale,
      "{:.2f}m".format(distance),
      ft.TextStyle(weight=ft.FontWeight.NORMAL, size=self.text_size, color=ft.colors.WHITE),
      alignment=ft.alignment.center,
      rotate=0.0 if self.top_down else math.pi
    )
    self.canvas.update()

  def build(self):
    self.max_distance = 1.0
    self.distance = 1.0
    self.fov = 30.0 / 180.0 * math.pi
    self.width = 144
    self.height = 200

    distance_scale = self.distance / self.max_distance
    path_height = self.height - self.text_size

    self.canvas = cv.Canvas([
        cv.Path([
            cv.Path.MoveTo(self.width / 2, 0),
            cv.Path.LineTo(
              math.sin(-self.fov / 2) * path_height * distance_scale + self.width / 2,
              math.cos(-self.fov / 2) * path_height * distance_scale),
            cv.Path.QuadraticTo(
              self.width / 2,
              path_height * distance_scale,
              math.sin(self.fov / 2) * path_height * distance_scale + self.width / 2,
              math.cos(self.fov / 2) * path_height * distance_scale),
            cv.Path.LineTo(self.width / 2, 0)
          ],
          paint=ft.Paint(
            style=ft.PaintingStyle.FILL,
            color=ft.colors.RED if self.top_down else ft.colors.GREEN
          )
        ),
        cv.Text(
          self.width / 2,
          (path_height + self.text_size / 2) * distance_scale,
          "nan",
          ft.TextStyle(weight=ft.FontWeight.NORMAL, size=self.text_size, color=ft.colors.WHITE),
          alignment=ft.alignment.center,
          rotate=0.0 if self.top_down else math.pi
        )
      ],
      width=self.width,
      height=self.height,
      rotate=0.0 if self.top_down else math.pi
    )

    return self.canvas

class EduardStatusReportDisplay(ft.UserControl):
  def __init__(self):
    super().__init__()

  def build(self):
    self.distance_front_left = RangeSensorDisplay(top_down=False)
    self.distance_front_right = RangeSensorDisplay(top_down=False)
    self.distance_rear_left = RangeSensorDisplay(top_down=True)
    self.distance_rear_right = RangeSensorDisplay(top_down=True)
    self.eduard_image = ft.Image(
      # src=f"images/eduard_offroad_top.png",
      src_base64=get_offroad_image(),
      fit=ft.ImageFit.CONTAIN,
      width=300,
      height=300)
    
    return ft.Column([
      ft.Row(
        [
          self.distance_front_left,
          self.distance_front_right
        ],
        alignment=ft.alignment.center
      ),
      self.eduard_image,
      ft.Row(
        [
          self.distance_rear_left,
          self.distance_rear_right
        ],
        alignment=ft.alignment.center
      )      
    ])
  
  def show_offroad(self):
    # self.eduard_image.src = f"images/eduard_offroad_top.png"
    self.eduard_image.src_base64=get_offroad_image()
    self.eduard_image.update()
    pass

  def show_mecanum(self):
    # self.eduard_image.src = f"images/eduard_mecanum_top.png"
    self.eduard_image.src_base64=get_mecanum_image() 
    self.eduard_image.update()
    pass

  def set_distance(self, range_msg, sensor_index : RangeSensor):
    if sensor_index is RangeSensor.FrontLeft:
      self.distance_front_left.set_distance(range_msg.range)
    if sensor_index is RangeSensor.FrontRight:
      self.distance_front_right.set_distance(range_msg.range)
    if sensor_index is RangeSensor.RearLeft:
      self.distance_rear_left.set_distance(range_msg.range)
    if sensor_index is RangeSensor.RearRight:
      self.distance_rear_right.set_distance(range_msg.range)

