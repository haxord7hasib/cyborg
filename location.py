import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

# Replace with the actual URLs of your ESP32 devices
esp32_a_url = 'http://192.168.0.101/rssi'
esp32_b_url = 'http://192.168.0.100/rssi'

# Set up a Requests session with retries
session = requests.Session()
retries = Retry(total=5, backoff_factor=0.1, status_forcelist=[500, 502, 503, 504])
session.mount('http://', HTTPAdapter(max_retries=retries))

# Function to get RSSI from ESP32 devices
def get_rssi(url):
    try:
        response = session.get(url, timeout=5)
        if response.status_code == 200:
            return int(response.text)
        else:
            print(f"Error: Received status code {response.status_code}")
    except requests.RequestException as e:
        print(f"Request failed: {e}")
    return None

# Function to convert RSSI to distance
# This function needs to be calibrated for your specific environment and hardware
def rssi_to_distance(rssi):
    # Placeholder calibration values: these need to be determined experimentally
    rssi_at_one_meter = -50  # Example: RSSI value at one meter from the ESP32
    path_loss_exponent = 2.0  # Example: Environmental factor for signal loss
    # Convert RSSI to meters, then to centimeters
    distance_meters = 10 ** ((rssi_at_one_meter - rssi) / (10 * path_loss_exponent))
    distance_cm = distance_meters * 100  # Convert meters to centimeters
    return distance_cm

# Initialize the plot with a scale appropriate for a 200 square foot area
# Assuming a square room for simplicity: sqrt(200) feet per side
# Convert feet to centimeters for the plot (1 foot = 30.48 cm)
side_length_cm = np.sqrt(200) * 30.48
fig, ax = plt.subplots(figsize=(10, 10))  # Making the plot bigger for better visualization
ax.set_xlim(-side_length_cm / 2, side_length_cm / 2)
ax.set_ylim(-side_length_cm / 2, side_length_cm / 2)
ax.grid(True)

# Plot initial positions of the router and ESP32 A
router_coords = (0, 0)
esp32_a_coords = (side_length_cm / 4, 0)  # Placing ESP32 A at 1/4 width of the room
ax.plot(*router_coords, 'ro', markersize=10, label='Router')  # Static red dot for the router
ax.plot(*esp32_a_coords, 'o', color='orange', markersize=10, label='ESP32 A')  # Static orange dot for the ESP32 A
esp32_b_point, = ax.plot([], [], 'go', markersize=10, label='ESP32 B')  # Moving ESP32 B point

# Text annotation for distances
distance_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def animate(i):
    # Fetch new RSSI data
    rssi_b = get_rssi(esp32_b_url)

    # Convert RSSI to distance
    distance_to_router = rssi_to_distance(rssi_b) if rssi_b else 0

    # Update the text annotation with the distance
    distance_text.set_text(f"Distance to Router: {distance_to_router:.2f} cm")

    # For simplicity, let's assume ESP32 B is moving along a line y=x
    # This is just for illustration; you'd calculate the actual position based on your own logic
    esp32_b_point.set_data(distance_to_router, distance_to_router)

    ax.relim()  # Recalculate limits
    ax.autoscale_view()  # Autoscale

# Create animation
ani = FuncAnimation(fig, animate, interval=1000, cache_frame_data=False)

plt.legend()
plt.show()
