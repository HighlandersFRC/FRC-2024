from kivy.uix.boxlayout import BoxLayout

from widgets.field_display import FieldDisplay
from widgets.menu import Menu
from tools import constants

import math

class OdomTracker(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation = "vertical", **kwargs)
        
        self.field_display = FieldDisplay(size = constants.FIELD_DIMENSIONS_PIXELS_IN_FRAME)
        self.menu = Menu(self.field_display.set_trail_length, size_hint = (1, (1 - constants.DISPLAY_SCALAR) / constants.DISPLAY_SCALAR))
        self.add_widget(self.menu)
        self.add_widget(self.field_display)

    def update(self, dt: float):
        self.field_display.draw_trail(dt)