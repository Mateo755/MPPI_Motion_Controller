import pygame
import numpy as np


# --- SPRITE WIZUALIZACJI ---
class VehicleSprite(pygame.sprite.Sprite):
    def __init__(self, model, start_pos, scale=50,  origin_x=400, origin_y=300):
        super().__init__()
        self.model = model
        self.scale = scale
        self.origin_x = origin_x
        self.origin_y = origin_y

        #self.state = np.zeros(8)  # s, n, mu, vx, vy, r, delta, T
        # Przykład: postęp 0, bok 0, kąt 0, vx = 2 m/s, reszta 0
        #self.state = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.2])

        L = 0.9 + 0.9  # lF + lR
        R = 2.5        # promień okręgu w metrach
        delta = np.arctan(L / R)

        self.state = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, delta, 0])


        self.control = np.array([0.0, 0.0])

        self.original_image = pygame.Surface((40, 20), pygame.SRCALPHA)
        pygame.draw.rect(self.original_image, (0, 0, 0), (0, 0, 40, 20), 2)
        self.image = self.original_image.copy()
        self.rect = self.image.get_rect(center=start_pos)

    def update(self, dt):
        dx = self.model.dynamics(self.state, self.control) * dt
        self.state += dx
        
        x = self.origin_x + self.state[0] * self.scale  # postęp do przodu
        y = self.origin_y + self.state[1] * self.scale  # przesunięcie boczne

        angle = -np.degrees(self.state[2])

        self.image = pygame.transform.rotate(self.original_image, angle)
        self.rect = self.image.get_rect(center=(x, y))

    def set_control(self, throttle, steering):
        self.control = np.array([steering, throttle])