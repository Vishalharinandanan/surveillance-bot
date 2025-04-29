import math
import numpy as np
import serial
import time

class DifferentialDriveRobot:
    def __init__(self):
        self.wheel_radius = 0.05  # 5 cm
        self.robot_width = 0.30  # Distance between wheels
        self.position = np.array([0.0, 0.0])
        self.orientation = 0.0  # In radians

    def inverse_kinematics(self, x_dot, y_dot):
        cos_psi = np.cos(self.orientation)
        sin_psi = np.sin(self.orientation)

        u = x_dot * cos_psi + y_dot * sin_psi
        v = -x_dot * sin_psi + y_dot * cos_psi
        r = (y_dot * cos_psi - x_dot * sin_psi) / (self.robot_width / 2)
        w1 = (20 * u) - (3 * r)  # Left wheel rad/s
        w2 = (20 * u) + (3 * r)  # Right wheel rad/s

        rpm_left = w1 * 60 / (2 * math.pi)
        rpm_right = w2 * 60 / (2 * math.pi)

        return rpm_left, rpm_right

    def update_position(self, linear_vel, angular_vel, dt):
        dx = linear_vel * math.cos(self.orientation)
        dy = linear_vel * math.sin(self.orientation)

        self.position[0] += dx * dt
        self.position[1] += dy * dt
        self.orientation += angular_vel * dt

    def get_position(self):
        return self.position, self.orientation


def send_rpm_to_arduino(ser, rpm_left, rpm_right):
    try:
        message = f"{rpm_left:.2f},{rpm_right:.2f}\n"
        ser.write(message.encode())
        print(f"Sent RPM - Left: {rpm_left:.2f}, Right: {rpm_right:.2f}")
    except Exception as e:
        print("Error sending to Arduino:", e)


def navigate_to_waypoints():
    robot = DifferentialDriveRobot()
    dt = 0.1
    max_linear_velocity = 0.4
    min_turn_ratio = 0.4  # How slow the inner wheel should go when turning

    waypoints = [
    np.array([0.00, 0.00]),
    np.array([5.00, 0.00]),
    np.array([5.00, 5.00]),
    np.array([0.00, 5.00]),
    np.array([0.00, 0.00])
    ]


    try:
        ser = serial.Serial('COM3', 9600, timeout=1)
        time.sleep(2)
    except serial.SerialException as e:
        print("Failed to connect to serial port:", e)
        return

    while True:
        for target in waypoints:
            print(f"Navigating to waypoint: ({target[0]}, {target[1]})")

            while True:
                position, theta = robot.get_position()
                direction = target - position
                distance = np.linalg.norm(direction)

                if distance < 0.2:
                    send_rpm_to_arduino(ser, 0, 0)
                    print(f"Reached waypoint: ({target[0]}, {target[1]})\n")
                    break

                desired_theta = np.arctan2(direction[1], direction[0])
                angle_diff = (desired_theta - theta + np.pi) % (2 * np.pi) - np.pi

                # Forward velocity decreases slightly with large angle_diff
                velocity = max_linear_velocity * max(0.4, 1 - abs(angle_diff) / (np.pi / 2))

                # Asymmetric turning: both wheels forward, but at different speeds
                if angle_diff > 0:  # Turn left
                    left_scale = min_turn_ratio
                    right_scale = 1.0
                else:  # Turn right
                    left_scale = 1.0
                    right_scale = min_turn_ratio

                # Scale velocities
                v_left = velocity * left_scale
                v_right = velocity * right_scale

                # Convert to RPMs
                w_l = v_left / robot.wheel_radius
                w_r = v_right / robot.wheel_radius
                rpm_left = w_l * 60 / (2 * math.pi)
                rpm_right = w_r * 60 / (2 * math.pi)

                send_rpm_to_arduino(ser, rpm_left, rpm_right)
                avg_velocity = (v_left + v_right) / 2
                angular_velocity = (v_right - v_left) / robot.robot_width
                robot.update_position(avg_velocity, angular_velocity, dt)

                time.sleep(dt)

    send_rpm_to_arduino(ser, 0, 0)
    ser.close()





if __name__ == "__main__":
    navigate_to_waypoints()
