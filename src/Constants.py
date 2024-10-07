from random import randint

KP = 0.9  # Proportional gain for PID
KI = 0.1  # Integral gain for PID
KD = 0.05  # Derivative gain for PID
PID_OUTPUT_LIMITS = (-5, 5)  # Limits for control surface adjustments (in degrees)