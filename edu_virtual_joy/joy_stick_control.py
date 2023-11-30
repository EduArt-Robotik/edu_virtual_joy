import flet as ft
import flet.canvas as cv

import math

class JoyStickControl(ft.UserControl):
  def resizing(self, e):
    print('resizing')
    print(e)
    self.canvas.shapes[0].x = e.width / 2
    self.canvas.shapes[0].y = e.height / 2
    self.canvas.shapes[1].x = e.width / 2
    self.canvas.shapes[1].y = e.height / 2

    self.canvas.width = e.width
    self.canvas.height = e.width

    self.canvas.update()

  def on_user_input(self, e):
    width = self.canvas.width
    height = self.canvas.height
    x = e.local_x - width / 2
    y = e.local_y - height / 2
    length = math.sqrt(x * x + y * y)

    if length > self.size / 2 - self.dot_radius:
      factor = (self.size / 2 - self.dot_radius) / float(length)
      x = float(x) * factor
      y = float(y) * factor

    self.x = x / float(self.size / 2)
    self.y = y / float(self.size / 2)
    print('x = ', x)
    print('y = ', y)

    self.canvas.shapes[0].x = x + width / 2
    self.canvas.shapes[0].y = y + height / 2
    self.canvas.update()

  def on_user_input_release(self, e):
    self.canvas.shapes[0].x = self.canvas.width / 2
    self.canvas.shapes[0].y = self.canvas.height / 2
    self.canvas.update()  

  def build(self):
    self.size = 300
    self.dot_radius = 30
    self.x = 0.0
    self.y = 0.0
    self.canvas = cv.Canvas([
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

    return self.canvas

  def value(self):
    return [self.x, self.y]