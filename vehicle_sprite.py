import pygame
import numpy as np

def wrap_angle(angle):
    # Zmienia kąt, żeby zawsze mieścił się w przedziale [-180, 180]
    return (angle + 180) % 360 - 180

class VehicleSprite:
    def __init__(self, model, start_pos, scale=50, origin_x=400, origin_y=300):
        self.model = model
        self.scale = scale
        self.origin_x = origin_x
        self.origin_y = origin_y

        # Stan: s, n, mu, vx, vy, r, delta, T
        L = 0.9 + 0.9
        R = 2.5
        delta = np.arctan(L / R)
        self.state = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, delta, 0])
        self.control = np.array([0.0, 0.0])

        # Rozmiary pojazdu
        self.car_width = 50
        self.car_height = 30

        # Korpus pojazdu
        self.original_body = pygame.Surface((self.car_width, self.car_height), pygame.SRCALPHA)
        pygame.draw.rect(self.original_body, (0, 0, 0), (0, 0, self.car_width, self.car_height), 2)

        # Zaznaczenie przodu auta (prawa krawędź)
        pygame.draw.line(self.original_body, (0, 0, 255), (self.car_width - 2, 0), (self.car_width - 2, self.car_height), 2)

        self.body_image = self.original_body.copy()
        self.body_rect = self.body_image.get_rect(center=start_pos)

        # Koła – przygotuj wycentrowany prostokąt
        self.wheel_width = 12
        self.wheel_height = 7
        self.wheel_surf = pygame.Surface((self.wheel_width, self.wheel_height), pygame.SRCALPHA)
        wheel_rect = self.wheel_surf.get_rect()
        pygame.draw.rect(self.wheel_surf, (20, 20, 20), wheel_rect.inflate(-2, -2))

        # Pozycje tylnych kół względem środka auta
        self.wheel_offsets = {
            "FL": ( self.car_width // 2 - 3,  self.car_height // 2),
            "FR": ( self.car_width // 2 - 3, -self.car_height // 2),
            "RL": (-self.car_width // 2 + 3,  self.car_height // 2),
            "RR": (-self.car_width // 2 + 3, -self.car_height // 2)
        }

        self.wheels = []

    def update(self, dt):
        dx = self.model.dynamics(self.state, self.control) * dt
        self.state += dx

        # Pozycja i orientacja korpusu
        x = self.origin_x + self.state[0] * self.scale
        y = self.origin_y + self.state[1] * self.scale
        car_angle = wrap_angle(-np.degrees(self.state[2]))

        # Obrót korpusu
        self.body_image = pygame.transform.rotate(self.original_body, car_angle)
        self.body_rect = self.body_image.get_rect(center=(x, y))

        # --- KOŁA ---
        self.wheels = []
        # Oblicz kąt dla przednich kół (skręt) – zależny od sterowania (delta)
        delta_deg = -np.degrees(self.state[6])  # skręt w stopniach dla przednich kół
 
        #steering_factor = 0.001  # Współczynnik ograniczający skręt

        theta = self.state[2]  # obrót korpusu w radianach
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        for key, (dx, dy) in self.wheel_offsets.items():
            local_offset = np.array([dx, dy])
            rotated_offset = R @ local_offset
            wheel_x = x + rotated_offset[0]
            wheel_y = y + rotated_offset[1]

            # Obrót tylnich kół = obrót korpusu / Przednie koła skręcają się o delta
            local_wheel_angle = (car_angle + delta_deg) if key in ("FL", "FR") else car_angle
            #print(local_wheel_angle)
            rotated_wheel = pygame.transform.rotate(self.wheel_surf, local_wheel_angle)
            rect = rotated_wheel.get_rect()
            rect.center = (wheel_x, wheel_y)

            self.wheels.append((rotated_wheel, rect))

    def draw(self, screen):
        screen.blit(self.body_image, self.body_rect)
        for img, rect in self.wheels:
            screen.blit(img, rect)

    def set_control(self, throttle, steering):
        self.control = np.array([steering, throttle])
