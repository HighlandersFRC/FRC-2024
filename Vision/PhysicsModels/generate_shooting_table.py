import math
import csv

# Constants
speedScaler = 1.46335968
gravity = 9.81
max_height = 3.048  # 10 ft
shooter_height = 0.4572  # 18 inches

distances = [4.445, 4.55, 4.65, 4.75, 4.85, 4.95, 5.05, 5.15, 5.25, 5.3, 5.4, 5.5, 5.6, 5.7, 5.8, 5.867, 5.95, 6.05, 6.15, 6.25, 6.35, 6.43, 6.5, 6.6, 6.7, 6.8, 6.9, 7.0, 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7, 7.8, 7.9, 8.0, 8.1, 8.2, 8.3, 8.4, 8.5, 8.6]  # Sample distances

def calculateAngle(distance, max_height):
    topRoot = math.sqrt(2 * gravity * (max_height - shooter_height))
    bottomRoot = (
        math.sqrt((2 * max_height) / gravity)
        + (math.sqrt(2 * gravity * (max_height - shooter_height)) / gravity)
    )
    denominator = distance / bottomRoot
    angle = math.atan(topRoot / denominator)
    return round((angle * (180 / math.pi)), 2)

def calculateSpeed(distance):
    first = 2 * gravity * (max_height - shooter_height)
    secondDenominator = (
        math.sqrt((2 * max_height) / gravity)
        + (math.sqrt(2 * gravity * (max_height - shooter_height)) / gravity)
    )
    second = math.pow((distance / secondDenominator), 2)
    speed = math.sqrt(first + second)
    adjustedSpeed = speed * speedScaler
    rpm = (60 * adjustedSpeed) / (0.1016 * math.pi) * 2
    return round(rpm, 2)

data = []

for distance in distances:
    angle = calculateAngle(distance, max_height)
    speed = calculateSpeed(distance)
    data.append([distance, angle, speed])

csv_filename = 'lob_shot_lookup_table.csv'

with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    
    csv_writer.writerow(['Distance (m)', 'Angle (degrees)', 'Speed (m/s)'])
    
    for row in data:
        csv_writer.writerow(row)

print(f"Data has been saved to {csv_filename}")
