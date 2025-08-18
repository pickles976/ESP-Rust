# TODO: plot the raw angle data and the Kalman filtered angle

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("./readings.csv")

print(df)
print(df.mean())
print(df.std())
print(df.var())

# Plot accel_raw and kalman_angle with t as the x-axis
plt.figure(figsize=(10, 6))
plt.plot(df['t'], df['accel_raw'], label='Accel Raw')
# plt.plot(df['t'], df['gyro_raw'], label='Gyro Raw')
plt.plot(df['t'], df['kalman_angle'], label='Kalman Angle')

# Add labels and title
plt.xlabel('Time (t)')
plt.ylabel('Value')
plt.title('Accel Raw and Kalman Angle over Time')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
