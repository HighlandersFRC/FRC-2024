from kivy.app import App
from kivy.clock import Clock
from kivy.config import Config
from kivy.core.window import Window

TITLE = "2024 Real-Time Odometry Tracker - Team 4499 The Highlanders"
ICON = "images/4499Icon.ico"

class OdomTrackerApp(App):
    def build(self):
        Clock.schedule_interval(self.execute, 1 / 60)
        self.title = TITLE
        self.icon = ICON
        

    def execute(self, dt):
        pass