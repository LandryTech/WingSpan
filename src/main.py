from random import randint
#import time
#import math
from simple_pid import PID  # For the PID loop
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import Constants

# === Constants (adjust as needed) ===
TARGET_ALTITUDE = randint(100,500)  # Target altitude for landing (0 means landing on ground level)
RUNWAY_DISTANCE = 400  # Distance to runway in meters (adjust based on actual scenario)

# Glider state
current_altitude = TARGET_ALTITUDE + randint(-100,300)  # Starting altitude
distance_to_target = 200  # Starting distance to runway (placeholder)
glider_angle = 0  # Initial glider angle (0 = horizontal)

# === Initialize Sensors (Placeholder) ===
def initialize_sensors():
    print("Sensors initialized (GPS, IMU, etc.)")
    return None  # Placeholder

# === PID Controller Setup ===
def initialize_pid():
    pid = PID(Kp=Constants.KP, Ki=Constants.KI, Kd=Constants.KD, setpoint=TARGET_ALTITUDE)
    pid.output_limits = Constants.PID_OUTPUT_LIMITS  # Limiting control output for smooth adjustments
    return pid

# === Target Glide Path Calculation ===
def calculate_glide_path(current_altitude, target_altitude, distance_to_target):
    if distance_to_target == 0:
        return target_altitude
    glide_slope = (current_altitude - target_altitude) / distance_to_target
    return glide_slope

# === Control Adjustment Based on PID ===
def adjust_controls(control_adjustment):
    """
    Adjust the glider angle based on the PID control output.
    For visualization, just update the angle.
    """
    global glider_angle
    glider_angle = control_adjustment  # Control the angle of the glider

# === Live Plot for Glider Angle and Target Altitude ===
def update_plot(frame):
    global current_altitude, distance_to_target, glider_angle

    # Increment altitude based on glider's angle
    altitude_change = glider_angle * 0.5  # Adjust factor (0.5) to control altitude change rate
    current_altitude += altitude_change
    
    # Ensure altitude doesn't go below the target altitude (landing simulation)
    #if current_altitude < TARGET_ALTITUDE:
    #    current_altitude = TARGET_ALTITUDE

    # === Calculate target glide path ===
    target_glide_path = calculate_glide_path(current_altitude, TARGET_ALTITUDE, distance_to_target)
    
    # Calculate the error (difference between current altitude and target glide slope)
    current_error = current_altitude - target_glide_path
    
    # === PID Control Adjustment ===
    control_adjustment = pid(current_error)
    
    # Apply the control adjustment to the glider (adjust angle)
    adjust_controls(control_adjustment)
    
    # Update the plot with new altitude and angle
    ax.clear()
    
    # Draw target altitude line (orange)
    ax.axhline(y=TARGET_ALTITUDE, color='orange', linestyle='--', label='Target Altitude')
    
    # Draw glider angle (blue line)
    ax.plot([0, 1], [current_altitude, current_altitude + glider_angle], color='blue', label='Glider Angle')
    
    # Add labels
    ax.set_ylim(-100, 600)  # Adjust to appropriate range based on altitude
    ax.set_xlim(0, 2)
    ax.axhline(y=0, color='green',label = 'Ground')
    ax.set_title(f"Glider Altitude: {current_altitude:.3f}, Angle: {glider_angle:.3f}°")
    ax.legend()

# === Setup Matplotlib for Live Plot ===
fig, ax = plt.subplots()

# Initialize PID controller
pid = initialize_pid()

# Animation function: call update_plot at regular intervals
ani = FuncAnimation(fig, update_plot, frames=range(100), interval=100)

# Show the plot
plt.show()
