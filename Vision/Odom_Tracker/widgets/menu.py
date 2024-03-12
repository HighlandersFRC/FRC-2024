from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from kivy.uix.label import Label

class Menu(GridLayout):
    def __init__(self, set_trail_func, sonic_func, **kwargs):
        super().__init__(rows = 2, orientation = "tb-lr", **kwargs)

        self.set_trail_func = set_trail_func

        self.trail_length_input = TextInput(text = "3", hint_text = "Trail Length (sec)", multiline = False, input_filter = "float", on_text_validate = self.set_trail_length, size_hint = (0.5, 1))
        self.trail_length_button = Button(text = "SET TRAIL LENGTH", on_press = self.set_trail_length, size_hint = (0.5, 1))
        self.add_widget(self.trail_length_input)
        self.add_widget(self.trail_length_button)

        self.connection_status_label = Label(text = "[b]Not Connected[/b]", markup = True, font_size = 20, color = (0.8, 0, 0))
        self.add_widget(self.connection_status_label)

        self.sonic_button = Button(text = "SONIC!", on_press = sonic_func)
        self.add_widget(self.sonic_button)

    def set_trail_length(self, event):
        length = float(self.trail_length_input.text)
        if length == "":
            return
        self.set_trail_func(length)

    def set_connection_status(self, connected: bool, address: str):
        if connected:
            self.connection_status_label.text = f"[b]Connected to roboRIO at {address}[/b]"
            self.connection_status_label.color = (0, 0.8, 0)
        else:
            self.connection_status_label.text = f"[b]Not Connected - Failed to connect to {address}[/b]"
            self.connection_status_label.color = (0.8, 0, 0)
        

    