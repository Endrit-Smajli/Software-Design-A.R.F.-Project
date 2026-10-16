#THIS IS THE ROBOT MOVEMENT (FIRST DRAFT)
import time
from adafruit_servokit import ServoKit

# PCA9685 servo controller: 16 channels
kit = ServoKit(channels=16)

# Example configuration: 3 servos per leg.
# Verify these channels against your actual wiring.
LEGS = {
    "front_left":  [0, 1, 2],
    "front_right": [3, 4, 5],
    "rear_left":   [6, 7, 8],
    "rear_right":  [9, 10, 11],
}

# Example neutral angles. These are NOT universal;
# calibrate each joint with the legs safely lifted.
NEUTRAL = [90, 90, 90]

def set_leg(leg, angles):
    """Set the three joint angles for one leg."""
    for channel, angle in zip(LEGS[leg], angles):
        kit.servo[channel].angle = angle

def stand():
    """Move all legs to their configured neutral pose."""
    for leg in LEGS:
        set_leg(leg, NEUTRAL)
    time.sleep(0.5)

def walk_forward():
    """
    Demonstration gait: alternate diagonal leg pairs.
    Tune swing angles and timing for your robot's geometry.
    """
    pairs = [
        ("front_left", "rear_right"),
        ("front_right", "rear_left"),
    ]

    for _ in range(4):
        for pair in pairs:
            # Swing one diagonal pair
            for leg in pair:
                set_leg(leg, [70, 100, 80])

            time.sleep(0.2)

            # Return the pair toward neutral
            for leg in pair:
                set_leg(leg, NEUTRAL)

            time.sleep(0.2)

def stop():
    """Return to the neutral pose; this is not an emergency power cut."""
    stand()

if __name__ == "__main__":
    try:
        stand()
        print("Robot ready. Press Ctrl+C to stop.")

        while True:
            walk_forward()
    except KeyboardInterrupt:
        print("\nStopping movement...")
        stop()

