import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# Simulate acceleration data
time = np.linspace(0, 10, 500)  # 10 seconds of data
acceleration = np.random.normal(0, 1, (3, time.shape[0]))  # Simulate 3-axis acceleration data

# Calculate roll, pitch, and yaw from acceleration data
roll = np.arctan2(acceleration[1], acceleration[2])
pitch = np.arctan2(-acceleration[0], np.sqrt(acceleration[1]**2 + acceleration[2]**2))
yaw = np.arctan2(acceleration[2], acceleration[1])

# Add noise to roll, pitch, and yaw
roll_noisy = roll + np.random.normal(0, 0.1, roll.shape)
pitch_noisy = pitch + np.random.normal(0, 0.1, pitch.shape)
yaw_noisy = yaw + np.random.normal(0, 0.1, yaw.shape)

# Apply Savitzky-Golay filter to simulate MKFF
window_length = 51  # Window length for the filter
polyorder = 3      # Polynomial order for the filter

roll_filtered = savgol_filter(roll_noisy, window_length, polyorder)
pitch_filtered = savgol_filter(pitch_noisy, window_length, polyorder)
yaw_filtered = savgol_filter(yaw_noisy, window_length, polyorder)

# Plotting
plt.figure(figsize=(15, 10))

# Plot Noisy Data
plt.subplot(3, 2, 1)
plt.plot(time, roll_noisy, label='Noisy Roll', color='blue')
plt.title('Noisy Roll')
plt.xlabel('Time (s)')
plt.ylabel('Roll (radians)')

plt.subplot(3, 2, 3)
plt.plot(time, pitch_noisy, label='Noisy Pitch', color='green')
plt.title('Noisy Pitch')
plt.xlabel('Time (s)')
plt.ylabel('Pitch (radians)')

plt.subplot(3, 2, 5)
plt.plot(time, yaw_noisy, label='Noisy Yaw', color='red')
plt.title('Noisy Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')

# Plot Filtered Data
plt.subplot(3, 2, 2)
plt.plot(time, roll_filtered, label='Filtered Roll', color='blue')
plt.title('Filtered Roll')
plt.xlabel('Time (s)')

plt.subplot(3, 2, 4)
plt.plot(time, pitch_filtered, label='Filtered Pitch', color='green')
plt.title('Filtered Pitch')
plt.xlabel('Time (s)')

plt.subplot(3, 2, 6)
plt.plot(time, yaw_filtered, label='Filtered Yaw', color='red')
plt.title('Filtered Yaw')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
