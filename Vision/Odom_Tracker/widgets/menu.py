from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from tools import nt_manager

class Menu(GridLayout):
    def __init__(self):
        self.rio_address_input = TextInput(hint_text = "RoboRIO Address", multiline = False, on_text_validate = self.connect_to_rio)
        self.rio_connect_button = Button(on_press = self.connect_to_rio)

    def connect_to_rio(self, event):
        pass