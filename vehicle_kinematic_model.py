import pygame
import numpy as np

# --- MODEL DYNAMIKI ---
class VehicleKinematicModel:
    """Klasa modelująca kinematykę pojazdu"""

    def __init__(self):
        self.lF = 0.9                                   # odległość od środka masy do osi przedniej
        self.lR = 0.9                                   # odległość od środka masy do osi tylnej
        self.wheel_base = self.lF + self.lR             # rozstaw osi
        self.max_steer_abs = 0.2036                     # maksymalny kąt skrętu (30°)
        self.max_accel_abs = 5.0                        # maksymalne przyspieszenie
        self.max_velocity = 5.0                         # maksymalna prędkość
        self.state = np.array([0.0, 0.0, 0.0, 1.0])     # wektor stanu -> x, y, yaw, v
        self.control = np.array([0.0, 0.0])


    def next_state(self, x, u, dt):
        """Zwraca nowy stan pojazdu na podstawie aktualnego stanu i sterowania"""

        steer, accel = u
        steer = np.clip(steer, -self.max_steer_abs, self.max_steer_abs)
        accel = np.clip(accel, -self.max_accel_abs, self.max_accel_abs)

        x_pos, y_pos, yaw, v = x

        dx = v * np.cos(yaw)
        dy = v * np.sin(yaw)
        dyaw = v / self.wheel_base * np.tan(steer)
        dv = accel
        v_new = v + dv * dt
        new_state = np.array([
            x_pos + dx * dt,
            y_pos + dy * dt,
            yaw + dyaw * dt,
            np.clip(v_new, -self.max_velocity, self.max_velocity)  # maksymalna prędkość
        ])

        return new_state

    
    def set_control(self, throttle, steering):
        """Ustawienie wektora sterowań"""
        self.control = np.array([steering, throttle])

