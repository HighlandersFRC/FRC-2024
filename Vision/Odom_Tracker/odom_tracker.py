from kivy.uix.boxlayout import BoxLayout

from widgets.field_display import FieldDisplay
from widgets.menu import Menu

import math

class OdomTracker(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation = "horizontal", **kwargs)

        self.window_size = (1920, 1012)
        self.display_scalar = 0.75
        
        self.field_display = FieldDisplay(size = (self.window_size[0] * self.display_scalar, self.window_size[1] * self.display_scalar))
        self.menu = Menu(size_hint = (2 * (1 - self.display_scalar), 1))
        self.add_widget(self.field_display)
        self.add_widget(self.menu)

    def update(self, dt: float):
        self.field_display.draw_trail(dt)