import pygame
import numpy as np
import torch
from torch_vehicle_dynamics import vehicle_dynamics, running_cost, terminal_cost, set_centerline
from pytorch_mppi import MPPI
from vehicle_sprite import VehicleSprite

SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900

# --- Półokrąg ---
num_points = 100
radius = 10.0
theta = np.linspace(0, np.pi, num_points)

# Generuj centerline z kolumnami: x, y, yaw, v
x_vals = radius * np.cos(theta)
y_vals = radius * np.sin(theta)
dx = np.gradient(x_vals)
dy = np.gradient(y_vals)
yaws = np.arctan2(dy, dx)
vs = np.full_like(x_vals, 4.0)  # stała prędkość referencyjna
centerline = np.column_stack((x_vals, y_vals, yaws, vs))

track_center = centerline[:, :2].mean(axis=0)

# Przekazanie referencji do running_cost
set_centerline(centerline)

scale = 20.0
origin_x = SCREEN_WIDTH // 2 - track_center[0] * scale
origin_y = SCREEN_HEIGHT // 2 - track_center[1] * scale

start_world = centerline[0, :2]
dir_vec = centerline[1, :2] - centerline[0, :2]
angle = np.arctan2(dir_vec[1], dir_vec[0])

def draw_track(screen, track, color):
    for point in track:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color, (x, y), 2)

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()

    start_px = int(origin_x + start_world[0] * scale)
    start_py = int(origin_y + start_world[1] * scale)
    car = VehicleSprite(None, start_pos=(start_px, start_py), origin_x=origin_x, origin_y=origin_y, scale=scale)
    car.state[0], car.state[1], car.state[2] = start_world[0], start_world[1], angle

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    u_init = torch.tensor([0.0, 0.1])

    mppi = MPPI(
        dynamics=vehicle_dynamics,
        running_cost=running_cost,
        terminal_state_cost=terminal_cost,
        nx=8,
        noise_sigma=torch.diag(torch.tensor([0.1, 0.1])),
        num_samples=500,
        horizon=20,
        lambda_=1.0,
        u_min=torch.tensor([-0.5, -0.2]),
        u_max=torch.tensor([0.5, 0.5]),
        u_init=u_init,
        device=device,
    )

    running = True
    while running:
        dt = clock.tick(60) / 1000.0 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        state_tensor = torch.tensor(car.state, dtype=torch.float32).to(device)
        u = mppi.command(state_tensor)

        next_state = vehicle_dynamics(state_tensor.unsqueeze(0), u.unsqueeze(0))[0]
        new_state = state_tensor + dt * next_state

        car.set_state_from_tensor(new_state)

        screen.fill((255, 255, 255))
        draw_track(screen, centerline[:, :2], (0, 0, 255))
        car.draw(screen)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
