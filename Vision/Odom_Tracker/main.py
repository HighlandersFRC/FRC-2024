from kivy.app import App
from kivy.clock import Clock
from kivy.config import Config
from kivy.core.window import Window
from odom_tracker import OdomTracker
import argparse

parser = argparse.ArgumentParser(description = "Tool for visualizing robot odometry in real time")
parser.add_argument("-a", "--address", required = False, default = "10.44.99.2")

args = parser.parse_args()

TITLE = "2024 Real-Time Odometry Tracker - Team 4499 The Highlanders"
ICON = "images/4499Icon.ico"

class OdomTrackerApp(App):
    def build(self):
        Clock.schedule_interval(self.execute, 1 / 60)
        self.title = TITLE
        self.icon = ICON
        self.odom_tracker = OdomTracker(args.address)
        Window.maximize()
        return self.odom_tracker

    def execute(self, dt):
        self.odom_tracker.update(dt)

if __name__ == "__main__":
    Config.set("input", "mouse", "mouse,multitouch_on_demand")
    OdomTrackerApp().run()