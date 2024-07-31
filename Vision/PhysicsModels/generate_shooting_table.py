import math
import csv

# Constants
speedScaler = 1.46335968
gravity = 9.81
max_height = 3.048  # 10 ft
shooter_height = 0.4572  # 18 inches

# Array of distances
distances = [5.867, 6.2, 6.5, 7.8, 8.2]  # Example distances

def calculateAngle(distance, max_height):
    topRoot = math.sqrt(2 * gravity * (max_height - shooter_height))
    bottomRoot = (
        math.sqrt((2 * max_height) / gravity)
        + (math.sqrt(2 * gravity * (max_height - shooter_height)) / gravity)
    )
    denominator = distance / bottomRoot
    angle = math.atan(topRoot / denominator)
    return angle * (180 / math.pi)

def calculateSpeed(distance):
    first = 2 * gravity * (max_height - shooter_height)
    secondDenominator = (
        math.sqrt((2 * max_height) / gravity)
        + (math.sqrt(2 * gravity * (max_height - shooter_height)) / gravity)
    )
    second = math.pow((distance / secondDenominator), 2)
    speed = math.sqrt(first + second)
    adjustedSpeed = speed * speedScaler
    return ((60 * adjustedSpeed) / (0.1016 * math.pi)) * 2

# Prepare data for CSV
data = []

# Iterate over the array of distances
for distance in distances:
    angle = calculateAngle(distance, max_height)
    speed = calculateSpeed(distance)
    data.append([distance, angle, speed])  # Append the results as a list

# Write data to a CSV file
csv_filename = 'lob_shot_lookup_table.csv'

with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    
    # Write header
    csv_writer.writerow(['Distance (m)', 'Angle (degrees)', 'Speed (m/s)'])
    
    # Write data rows
    for row in data:
        csv_writer.writerow(row)

print(f"Data has been saved to {csv_filename}")
