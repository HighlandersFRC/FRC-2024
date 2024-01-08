import math

from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.graphics import *
from tools import constants

class FieldDisplay(Image):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Trail of points (meters)
        self.trail_points = []
        # Most recent point
        self.current_point = None
        # Length of trail before ending (seconds)
        self.trail_length = 3

        self.field_image = Rectangle(source = "images/CrescendoField.png", pos = self.pos, size = self.size)

        self.info_label = Label(text = "[b]PX:[/b] null, [b]PY:[/b]\n[b]X:[/b] null, [b]Y:[/b] null", markup = True, font_size = 12, color = (0, 0, 0))
        self.info_rect = Rectangle()

    def draw_trail(self, dt: float):
        self.canvas.clear()

        self.canvas.add(Rectangle(pos = (0, 0), size = self.size))
        self.canvas.add(self.field_image)

        self.info_label.text = f"[b]PX:[/b] {constants.get_cursor_pos_pixels()[0]}, [b]PY: {constants.get_cursor_pos_pixels()[1]}[/b]\n[b]X:[/b] {round(constants.get_cursor_pos_meters()[0], 3)}, [b]Y:[/b] {round(constants.get_cursor_pos_meters()[1], 3)}"
        if self.info_label.texture != None:
            self.info_rect = Rectangle(texture = self.info_label.texture, size = list(self.info_label.texture.size), pos = (50, 700))
            self.canvas.add(self.info_rect)

    def set_trail_length(self, length: float):
        self.trail_length = length