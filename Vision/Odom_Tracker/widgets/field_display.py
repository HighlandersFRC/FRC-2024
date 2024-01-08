import math

from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.graphics import *
from tools import convert

class FieldDisplay(Image):
    def __init__(self, **kwargs):
        super().__init__(source = "images/CrescendoField.png", **kwargs)
        
        self.trail_points = []
        self.current_point = None

        self.field_image = Rectangle(source = "images/CrescendoField.png", pos = self.pos, size = self.size)

        self.info_label = Label(text = "[b]PX:[/b] null, [b]PY:[/b]\n[b]X:[/b] null, [b]Y:[/b] null\n[b]Dist:[/b] null", markup = True, font_size = 12, color = (0, 0, 0))
        self.info_rect = Rectangle()

    def draw_trail(self, dt: float):
        self.canvas.clear()

        self.canvas.add(Rectangle(pos = (0, 0), size = self.size))
        self.canvas.add(self.field_image)