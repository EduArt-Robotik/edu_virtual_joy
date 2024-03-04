import flet as ft
import flet.canvas as cv

import math

from edu_virtual_joy_ros_node import RangeSensor
from images import get_mecanum_image, get_offroad_image

class RangeSensorDisplay(ft.UserControl):
  def __init__(self, top_down : bool):
    super().__init__()
    self.top_down = top_down

  def set_distance(self, distance : float):
    print('set distance = ', distance)
    self.distance = min(distance, self.max_distance)
    distance_scale = self.distance / self.max_distance    

    # recalculate path elements based on new distance value
    self.canvas.shapes[0].elements[0] = cv.Path.MoveTo(self.width / 2, 0)
    self.canvas.shapes[0].elements[1] = cv.Path.LineTo(
      math.sin(-self.fov / 2) * self.height * distance_scale + self.width / 2,
      math.cos(-self.fov / 2) * self.height * distance_scale)
    self.canvas.shapes[0].elements[2] = cv.Path.QuadraticTo(
      self.width / 2,
      self.height * distance_scale,
      math.sin(self.fov / 2) * self.height * distance_scale + self.width / 2,
      math.cos(self.fov / 2) * self.height * distance_scale)
    self.canvas.shapes[0].elements[3] = cv.Path.LineTo(self.width / 2, 0)
    self.canvas.update()

  def build(self):
    self.max_distance = 1.0
    self.distance = 1.0
    self.fov = 30.0 / 180.0 * math.pi
    self.width = 200
    self.height = 300

    distance_scale = self.distance / self.max_distance

    self.canvas = cv.Canvas([
        cv.Path([
            cv.Path.MoveTo(self.width / 2, 0),
            cv.Path.LineTo(
              math.sin(-self.fov / 2) * self.height * distance_scale + self.width / 2,
              math.cos(-self.fov / 2) * self.height * distance_scale),
            cv.Path.QuadraticTo(
              self.width / 2,
              self.height * distance_scale,
              math.sin(self.fov / 2) * self.height * distance_scale + self.width / 2,
              math.cos(self.fov / 2) * self.height * distance_scale),
            cv.Path.LineTo(self.width / 2, 0)
          ],
          paint=ft.Paint(
            style=ft.PaintingStyle.FILL,
            color=ft.colors.GREEN
          )
        )
      ],
      width=self.width,
      height=self.height
    )

    return self.canvas

class EduardStatusReportDisplay(ft.UserControl):
  def __init__(self):
    super().__init__()

  def build(self):
    self.distance_front_left = RangeSensorDisplay(True)
    self.eduard_image = ft.Image(
      # src=f"images/eduard_offroad_top.png",
      src_base64=get_offroad_image(),
      fit=ft.ImageFit.CONTAIN,
      width=300,
      height=300)
    
    return ft.Column([
      self.eduard_image,
      self.distance_front_left
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