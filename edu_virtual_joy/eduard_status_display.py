import flet as ft
import flet.canvas as cv

import math

class RangeSensorDisplay(ft.UserControl):
  def build(self):
    self.distance = 1.0
    self.fov = 10.0 / 180.0 * math.pi
    self.width = 200
    self.height = 100

    self.canvas = cv.Canvas([
        cv.Path([
            cv.Path.MoveTo(self.width / 2, 0),
            cv.Path.LineTo(math.cos(-self.fov / 2), math.sin(-self.fov / 2)),
            cv.Path.QuadraticTo(math.cos(self.fov / 2), math.sin(self.fov / 2), math.cos(self.fov / 2), math.sin(self.fov / 2)),
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
  def build(self):
    self.eduard_image = ft.Image(src=f"images/eduard_offroad_top.png")
    self.distance_front_left = RangeSensorDisplay()

    return ft.Column([
      self.eduard_image,
      self.distance_front_left
    ])
  
  def show_offroad(self):
    self.eduard_image.src = f"images/eduard_offroad_top.png"
    self.eduard_image.update()

  def show_mecanum(self):
    self.eduard_image.src = f"images/eduard_mecanum_top.png"
    self.eduard_image.update()
