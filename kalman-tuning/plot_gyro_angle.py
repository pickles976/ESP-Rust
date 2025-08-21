# TODO: plot the raw angle data and the Kalman filtered angle

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("./readings.csv")

# Get angle from cumulative measurements
dt = 0.01
PI = 3.1415926

angular = df["gyro_raw"]

cumulative = []
for i in range(len(angular)):
    cumulative.append(dt * sum(angular[:i]))
df["angle"] = cumulative

# Plot accel_raw and kalman_angle with t as the x-axis
plt.figure(figsize=(10, 6))
plt.plot(df['t'], df['angle'], label='Angle')
plt.plot(df['t'], df['gyro_raw'], label='Gyro Raw', linestyle='dashed')

# Add labels and title
plt.xlabel('Time (t)')
plt.ylabel('Value')
plt.title('Accel Raw and Kalman Angle over Time')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
