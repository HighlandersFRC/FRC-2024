from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from tools import nt_manager

class Menu(GridLayout):
    def __init__(self, set_trail_func, **kwargs):
        super().__init__(rows = 2, orientation = "tb-lr", **kwargs)

        self.set_trail_func = set_trail_func

        self.rio_address_input = TextInput(hint_text = "RoboRIO Address", multiline = False, on_text_validate = self.connect_to_rio)
        self.rio_connect_button = Button(text = "CONNECT", on_press = self.connect_to_rio)
        self.add_widget(self.rio_address_input)
        self.add_widget(self.rio_connect_button)

        self.trail_length_input = TextInput(hint_text = "Trail Length (sec)", multiline = False, input_filter = "float", on_text_validate = self.set_trail_length)
        self.trail_length_button = Button(text = "SET", on_press = self.set_trail_length)
        self.add_widget(self.trail_length_input)
        self.add_widget(self.trail_length_button)

    def connect_to_rio(self, event):
        address = float(self.rio_address_input.text)
        if address == "":
            return

    def set_trail_length(self, event):
        length = float(self.trail_length_input.text)
        if length == "":
            return
        self.set_trail_func(length)