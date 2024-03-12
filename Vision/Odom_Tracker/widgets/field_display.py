import math

from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.graphics import *
from tools import constants
import random
import json

class FieldDisplay(Image):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Trail of points (meters)
        self.trail_points = []
        # Most recent point
        self.current_point = None
        # Length of trail before ending (seconds)
        self.trail_length = 3

        self.current_tracks = []

        self.field_image = Rectangle(source = "images/CrescendoField.png", pos = self.pos, size = self.size)

        self.info_label = Label(text = "[b]PX:[/b] null, [b]PY:[/b]\n[b]X:[/b] null, [b]Y:[/b] null", markup = True, font_size = 12, color = (0, 0, 0))
        self.info_rect = Rectangle()

        self.sonic = False

    def draw_trail(self, dt: float):
        self.canvas.clear()

        # Field background
        self.canvas.add(Rectangle(pos = (0, 0), size = self.size))
        self.canvas.add(self.field_image)

        # Info lable
        if len(self.trail_points) == 0:
            self.info_label.text = f"[b]PX:[/b] {constants.get_cursor_pos_pixels()[0]}, [b]PY: {constants.get_cursor_pos_pixels()[1]}[/b]\n[b]X:[/b] {round(constants.get_cursor_pos_meters()[0], 3)}, [b]Y:[/b] {round(constants.get_cursor_pos_meters()[1], 3)}\n[b]Length:[/b] {self.trail_length} sec."
        else:
            track_str = "".join([f"{self.current_tracks[i]['fID']}, " if i != len(self.current_tracks) - 1 else f"{self.current_tracks[i]['fID']}" for i in range(len(self.current_tracks))])
            self.info_label.text = f"[b]PX:[/b] {constants.get_cursor_pos_pixels()[0]}, [b]PY: {constants.get_cursor_pos_pixels()[1]}[/b]\n[b]X:[/b] {round(constants.get_cursor_pos_meters()[0], 3)}, [b]Y:[/b] {round(constants.get_cursor_pos_meters()[1], 3)}\n[b]Length:[/b] {self.trail_length} sec.\n[b]Robot:[/b] ({round(self.trail_points[-1][1], 3)}, {round(self.trail_points[-1][2], 3)})\n[b]Heading:[/b] {self.trail_points[-1][3] * 180 / math.pi} deg.\n[b]Tracks:[/b] {track_str}"
        if self.info_label.texture != None:
            self.info_rect = Rectangle(texture = self.info_label.texture, size = list(self.info_label.texture.size), pos = (50, 800))
            self.canvas.add(self.info_rect)

        # Trail
        for odom in self.trail_points:
            if abs(self.trail_points[-1][0] - odom[0]) > self.trail_length:
                self.trail_points.remove(odom)
        # if len(self.trail_points) > 0:
        #     print(self.trail_points[-1])
        pixel_list = []
        for odom in self.trail_points:
            pixel_list.append(constants.meters_to_pixels_x(odom[1]))
            pixel_list.append(constants.meters_to_pixels_y(odom[2]))
        self.canvas.add(Color(0, 0, 0, 1))
        self.canvas.add(Line(points = pixel_list, width = 2, cap = "round", joint = "round"))

        if len(self.trail_points) > 0:
            pose = self.trail_points[-1]
            for track in self.current_tracks:
                camera_pos = (0, 0)
                fid = track["fID"]
                if track["camera"] == "back_cam":
                    camera_pos = (pose[1] + constants.BACK_CAMERA_POSITION_POLAR[0] * math.cos(constants.BACK_CAMERA_POSITION_POLAR[1] + pose[3]), pose[2] + constants.BACK_CAMERA_POSITION_POLAR[0] * math.sin(constants.BACK_CAMERA_POSITION_POLAR[1] + pose[3]))
                elif track["camera"] == "front_cam":
                    camera_pos = (pose[1] + constants.FRONT_CAMERA_POSITION_POLAR[0] * math.cos(constants.FRONT_CAMERA_POSITION_POLAR[1] + pose[3]), pose[2] + constants.FRONT_CAMERA_POSITION_POLAR[0] * math.sin(constants.FRONT_CAMERA_POSITION_POLAR[1] + pose[3]))
                elif track["camera"] == "left_cam":
                    camera_pos = (pose[1] + constants.LEFT_CAMERA_POSITION_POLAR[0] * math.cos(constants.LEFT_CAMERA_POSITION_POLAR[1] + pose[3]), pose[2] + constants.LEFT_CAMERA_POSITION_POLAR[0] * math.sin(constants.LEFT_CAMERA_POSITION_POLAR[1] + pose[3]))
                elif track["camera"] == "right_cam":
                    camera_pos = (pose[1] + constants.RIGHT_CAMERA_POSITION_POLAR[0] * math.cos(constants.RIGHT_CAMERA_POSITION_POLAR[1] + pose[3]), pose[2] + constants.RIGHT_CAMERA_POSITION_POLAR[0] * math.sin(constants.RIGHT_CAMERA_POSITION_POLAR[1] + pose[3]))
                camera_pos_pixels = constants.meters_to_pixels(camera_pos)
                self.canvas.add(Color(0, 0.8, 0, 1))
                self.canvas.add(Line(points = [camera_pos_pixels[0], camera_pos_pixels[1], constants.meters_to_pixels_x(constants.TAG_POSES[fid - 1][0]), constants.meters_to_pixels_y(constants.TAG_POSES[fid - 1][1])], width = 2, cap = "round", joint = "round"))

        # Robot
        if len(self.trail_points) > 0:
            pose = self.trail_points[-1]
            corner_1 = constants.meters_to_pixels((constants.CORNER_RADIUS * math.cos(constants.CORNER_1_ANGLE + pose[3]) + pose[1], constants.CORNER_RADIUS * math.sin(constants.CORNER_1_ANGLE + pose[3]) + pose[2]))
            corner_2 = constants.meters_to_pixels((constants.CORNER_RADIUS * math.cos(constants.CORNER_2_ANGLE + pose[3]) + pose[1], constants.CORNER_RADIUS * math.sin(constants.CORNER_2_ANGLE + pose[3]) + pose[2]))
            corner_3 = constants.meters_to_pixels((constants.CORNER_RADIUS * math.cos(constants.CORNER_3_ANGLE + pose[3]) + pose[1], constants.CORNER_RADIUS * math.sin(constants.CORNER_3_ANGLE + pose[3]) + pose[2]))
            corner_4 = constants.meters_to_pixels((constants.CORNER_RADIUS * math.cos(constants.CORNER_4_ANGLE + pose[3]) + pose[1], constants.CORNER_RADIUS * math.sin(constants.CORNER_4_ANGLE + pose[3]) + pose[2]))
            self.canvas.add(Color(0.8, 0, 0.8, 1))
            self.canvas.add(Line(points = [corner_1[0], corner_1[1], corner_2[0], corner_2[1], corner_3[0], corner_3[1], corner_4[0], corner_4[1]], width = 2, cap = "square", joint = "miter", close = True))
            front_line = constants.meters_to_pixels((constants.ROBOT_LENGTH_METERS * 0.5 * math.cos(pose[3]) + pose[1], constants.ROBOT_LENGTH_METERS * 0.5 * math.sin(pose[3]) + pose[2]))
            robot_center = constants.meters_to_pixels((pose[1], pose[2]))
            self.canvas.add(Line(points = [robot_center[0], robot_center[1], front_line[0], front_line[1]], width = 2, cap = "square", joint = "miter"))
            if self.sonic:
                sonic_size = (100, 100)
                sonic = Rectangle(source = "images/sonic.png", pos = (robot_center[0] - sonic_size[0] / 2, robot_center[1] - sonic_size[1] / 2), size = sonic_size)
                self.canvas.add(Color(1, 1, 1, 1))
                self.canvas.add(Rotate(origin = robot_center, angle = constants.rad_to_deg(pose[3])))
                self.canvas.add(sonic)

    def set_trail_length(self, length: float):
        self.trail_length = length

    def update_trail_values(self, odom: str):
        odom_data = json.loads(odom)
        pose = [odom_data["time"], odom_data["pose"]["x"], odom_data["pose"]["y"], odom_data["pose"]["theta"]]
        self.trail_points.append(pose)
        self.current_tracks = odom_data["tracks"]
        # print(pose)

    def toggle_sonic(self, event):
        if self.sonic == False:
            self.sonic = True
        else:
            self.sonic = False