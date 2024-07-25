import math
import pandas as pd

def calculate_shooting_parameters(distances, max_height):
    # Constants
    g = 9.81  # Acceleration due to gravity (m/s^2)

    # Data storage
    results = []

    # Iterate over each target distance
    for distance in distances:
        best_angle = None
        best_speed = None

        # Iterate over possible angles from 0 to 90 degrees
        for angle in range(1, 90):
            theta = math.radians(angle)

            # Calculate required speed for this angle and distance
            # Using the horizontal range formula: distance = (v^2 * sin(2 * theta)) / g
            required_speed = math.sqrt(distance * g / math.sin(2 * theta))

            # Calculate maximum height with this angle and speed
            max_height_for_angle = (required_speed**2 * math.sin(theta)**2) / (2 * g)

            # Check if the maximum height condition is satisfied
            if max_height_for_angle >= max_height:
                best_angle = angle
                best_speed = required_speed
                break

        if best_angle is not None and best_speed is not None:
            results.append({
                'Distance (m)': distance,
                'Angle (degrees)': best_angle,
                'Speed (m/s)': best_speed
            })
        else:
            results.append({
                'Distance (m)': distance,
                'Angle (degrees)': 'N/A',
                'Speed (m/s)': 'N/A'
            })

    return results

# Define distances and maximum height requirement
distances = [50, 100, 150, 200, 250]  # Example distances in meters
max_height = 20  # Required height in the middle in meters

# Calculate shooting parameters
shooting_parameters = calculate_shooting_parameters(distances, max_height)

# Create a DataFrame and print it
df = pd.DataFrame(shooting_parameters)
print(df)

# Optionally, save the results to a CSV file
df.to_csv('shooting_parameters.csv', index=False)
