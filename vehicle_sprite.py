import pygame
import numpy as np
from math import degrees


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class VehicleSprite:
    def __init__(self, model, start_pos, scale=50, origin_x=400, origin_y=300):
        self.model = model
        self.scale = scale
        self.origin_x = origin_x
        self.origin_y = origin_y

        # --- STAN POJAZDU ---
        self.state = np.array([0.0, 0.0, 0.0,   # s, n, mu (nieużywane)
                               0.0, 0.0, 0.0,   # vx, vy, r
                               0.0, 0.0])      # delta, T
        self.control = np.array([0.0, 0.0])     # [delta_dot, T_dot]

        # Własna pozycja (niezależna od toru)
        self.x_pos = 0.0  # [m]
        self.y_pos = 0.0  # [m]
        self.yaw = 0.0    # [rad]

        # --- WYGLĄD ---
        self.car_width = 50
        self.car_height = 30

        self.original_body = pygame.Surface((self.car_width, self.car_height), pygame.SRCALPHA)
        pygame.draw.rect(self.original_body, (0, 0, 0), (0, 0, self.car_width, self.car_height), 2)
        pygame.draw.line(self.original_body, (0, 0, 255), (self.car_width - 2, 0), (self.car_width - 2, self.car_height), 2)
        self.body_image = self.original_body.copy()
        self.body_rect = self.body_image.get_rect(center=start_pos)

        # --- KOŁA ---
        self.wheel_width = 12
        self.wheel_height = 6
        self.wheel_surf = pygame.Surface((self.wheel_width, self.wheel_height), pygame.SRCALPHA)
        pygame.draw.rect(self.wheel_surf, (20, 20, 20), self.wheel_surf.get_rect().inflate(-2, -2))

        self.wheel_offsets = {
            "RL": (-self.car_width // 2 + 3,  self.car_height // 2),
            "RR": (-self.car_width // 2 + 3, -self.car_height // 2),
            "FL": ( self.car_width // 2 - 3,  self.car_height // 2),
            "FR": ( self.car_width // 2 - 3, -self.car_height // 2),
        }

        self.wheels = []

    def apply_control_input(self, throttle_input, steering_input, dt):
        max_delta_rate = np.radians(90)
        max_throttle_rate = 2.0

        delta_dot = max_delta_rate * steering_input

        if throttle_input > 0:
            T_dot = max_throttle_rate
        elif throttle_input < 0:
            T_dot = -max_throttle_rate
        else:
            # Naturalne wytracanie
            if self.state[7] > 0:
                T_dot = -max_throttle_rate
            elif self.state[7] < 0:
                T_dot = max_throttle_rate
            else:
                T_dot = 0.0

        self.control = np.array([delta_dot, T_dot])

    def update(self, dt):
        dx = self.model.dynamics(self.state, self.control) * dt
        self.state += dx

        # Integracja pozycji i yaw z vx, vy, r
        vx, vy, r = self.state[3], self.state[4], self.state[5]
        self.yaw += r * dt
        self.yaw = wrap_angle(self.yaw)

        self.x_pos += (vx * np.cos(self.yaw) - vy * np.sin(self.yaw)) * dt
        self.y_pos += (vx * np.sin(self.yaw) + vy * np.cos(self.yaw)) * dt

        # Rysowanie korpusu
        screen_x = self.origin_x + self.x_pos * self.scale
        screen_y = self.origin_y - self.y_pos * self.scale
        car_angle = -degrees(self.yaw)

        self.body_image = pygame.transform.rotate(self.original_body, car_angle)
        self.body_rect = self.body_image.get_rect(center=(screen_x, screen_y))

        # KOŁA
        self.wheels = []
        delta_deg = degrees(self.state[6])
        for key, (dx, dy) in self.wheel_offsets.items():
            R = np.array([
                [np.cos(self.yaw), -np.sin(self.yaw)],
                [np.sin(self.yaw),  np.cos(self.yaw)]
            ])
            local_offset = np.array([dx, dy])
            rotated_offset = R @ local_offset
            wheel_x = screen_x + rotated_offset[0]
            wheel_y = screen_y + rotated_offset[1]

            local_angle = car_angle + (delta_deg if key in ("FL", "FR") else 0)
            rotated_wheel = pygame.transform.rotate(self.wheel_surf, local_angle)
            rect = rotated_wheel.get_rect(center=(wheel_x, wheel_y))
            self.wheels.append((rotated_wheel, rect))

    def draw(self, screen):
        screen.blit(self.body_image, self.body_rect)
        for img, rect in self.wheels:
            screen.blit(img, rect)

    def stop(self):
        self.state[3] = 0.0  # vx
        self.state[4] = 0.0  # vy
        self.state[5] = 0.0  # r
        self.state[7] = 0.0  # T
        self.control = np.array([0.0, 0.0])

    def respawn(self, pos=None):
        if pos is None:
            pos = (self.origin_x, self.origin_y)
        self.state = np.zeros(8)
        self.control = np.zeros(2)
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.body_rect.center = pos
