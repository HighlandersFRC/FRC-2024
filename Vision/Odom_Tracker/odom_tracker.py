from kivy.uix.boxlayout import BoxLayout

from widgets.field_display import FieldDisplay
from widgets.menu import Menu
from tools import constants
from tools.nt_manager import NTManager

import math

class OdomTracker(BoxLayout):
    def __init__(self, rio_address: str, **kwargs):
        super().__init__(orientation = "vertical", **kwargs)

        self.field_display = FieldDisplay(size = constants.FIELD_DIMENSIONS_PIXELS_IN_FRAME)
        self.menu = Menu(self.field_display.set_trail_length, self.field_display.toggle_sonic, size_hint = (1, (1 - constants.DISPLAY_SCALAR) / constants.DISPLAY_SCALAR))
        self.add_widget(self.menu)
        self.add_widget(self.field_display)

        self.rio_address = rio_address
        self.nt_manager = NTManager(self.update_value, self.menu.set_connection_status, self.rio_address)

    def update(self, dt: float):
        self.field_display.draw_trail(dt)

    def update_value(self, key: str, value):
        if key == "odometry_data":
            self.field_display.update_trail_values(value)