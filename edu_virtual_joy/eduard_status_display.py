import flet as ft

class EduardStatusReportDisplay(ft.UserControl):
  def build(self):
    self.eduard_image = ft.Image(src=f"images/eduard_offroad_top.png")

    return self.eduard_image