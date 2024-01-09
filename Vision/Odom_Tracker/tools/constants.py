import math
import pyautogui

FIELD_IMAGE_WIDTH_PIXELS = 2601
FIELD_IMAGE_HEIGHT_PIXELS = 1291

FIELD_WIDTH_PIXELS = FIELD_IMAGE_WIDTH_PIXELS
FIELD_HEIGHT_PIXELS = FIELD_IMAGE_HEIGHT_PIXELS

FIELD_WIDTH_METERS = 16.59128
FIELD_HEIGHT_METERS = 8.211312

FIELD_WIDTH_OFFSET_PIXELS = 0
FIELD_HEIGHT_OFFSET_PIXELS = 0

WINDOW_WIDTH_PIXELS = 1920
WINDOW_HEIGHT_PIXELS = 1012

TASK_BAR_HEIGHT_PIXELS = 40
WINDOW_BAR_HEIGHT_PIXELS = 25

DISPLAY_SCALAR = 0.9418

ROBOT_WIDTH_METERS = 0.635
ROBOT_LENGTH_METERS = 0.7366

#   Front
# 2---x---1
# |   ^   |
# |y<-|   |
# |       |
# 3-------4

CORNER_1_OFFSET = (ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2)
CORNER_2_OFFSET = (ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2)
CORNER_3_OFFSET = (-ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2)
CORNER_4_OFFSET = (-ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2)

CORNER_1_ANGLE = math.atan2(CORNER_1_OFFSET[1], CORNER_1_OFFSET[0])
CORNER_2_ANGLE = math.atan2(CORNER_2_OFFSET[1], CORNER_2_OFFSET[0])
CORNER_3_ANGLE = math.atan2(CORNER_3_OFFSET[1], CORNER_3_OFFSET[0])
CORNER_4_ANGLE = math.atan2(CORNER_4_OFFSET[1], CORNER_4_OFFSET[0])

print(CORNER_1_ANGLE)
print(CORNER_2_ANGLE)
print(CORNER_3_ANGLE)
print(CORNER_4_ANGLE)

def get_dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

CORNER_RADIUS = get_dist(0, 0, CORNER_1_OFFSET[0], CORNER_1_OFFSET[1])

def get_field_dimensions(scalar: float) -> tuple[float]:
    if (FIELD_IMAGE_WIDTH_PIXELS / WINDOW_WIDTH_PIXELS) > (FIELD_IMAGE_HEIGHT_PIXELS / (WINDOW_HEIGHT_PIXELS * scalar)):
        return (WINDOW_WIDTH_PIXELS, FIELD_IMAGE_HEIGHT_PIXELS / (FIELD_IMAGE_WIDTH_PIXELS / WINDOW_WIDTH_PIXELS))
    else:
        return (FIELD_IMAGE_WIDTH_PIXELS / (FIELD_IMAGE_HEIGHT_PIXELS / (WINDOW_HEIGHT_PIXELS * scalar)), WINDOW_HEIGHT_PIXELS * scalar)
    
FIELD_DIMENSIONS_PIXELS_IN_FRAME = get_field_dimensions(DISPLAY_SCALAR)
FIELD_WIDTH_PIXELS_IN_FRAME = FIELD_DIMENSIONS_PIXELS_IN_FRAME[0]
FIELD_HEIGHT_PIXELS_IN_FRAME = FIELD_DIMENSIONS_PIXELS_IN_FRAME[1]

def meters_to_pixels_x(meters_x: float) -> float:
    return meters_x * (FIELD_WIDTH_PIXELS_IN_FRAME / FIELD_WIDTH_METERS)

def pixels_to_meters_x(pixels_x: float) -> float:
    return pixels_x * (FIELD_WIDTH_METERS / FIELD_WIDTH_PIXELS_IN_FRAME)

def meters_to_pixels_y(meters_y: float) -> float:
    return meters_y * (FIELD_HEIGHT_PIXELS_IN_FRAME / FIELD_HEIGHT_METERS)

def pixels_to_meters_y(pixels_y: float) -> float:
    return pixels_y * (FIELD_HEIGHT_METERS / FIELD_HEIGHT_PIXELS_IN_FRAME)

def meters_to_pixels(meters: tuple[float]) -> tuple[float]:
    return (meters_to_pixels_x(meters[0]), meters_to_pixels_y(meters[1]))

def pixels_to_meters(pixels: tuple[float]) -> tuple[float]:
    return (pixels_to_meters_x(pixels[0]), pixels_to_meters_y(pixels[1]))

def get_cursor_pos_pixels() -> tuple[float]:
    x, y = pyautogui.position()
    y = WINDOW_HEIGHT_PIXELS - y + WINDOW_BAR_HEIGHT_PIXELS
    return (x, y)

def get_cursor_pos_meters() -> tuple[float]:
    return pixels_to_meters(get_cursor_pos_pixels())

def deg_to_rad(deg: float):
    return (deg * math.pi) / 180

def rad_to_deg(deg: float):
    return (deg * 180) / math.pi

def in_to_m(inches: float):
    return inches / 39.37