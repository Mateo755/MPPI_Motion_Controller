import pygame
import sys
from vehicle_sprite import VehicleSprite
from vehicle_model import VehicleModelKinematic,VehicleModel  # <- Twój model dynamiki pojazdu

# --- MAIN ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((1200, 900))
    pygame.display.set_caption("Symulacja - WASD")
    clock = pygame.time.Clock()

    # Model i sprite pojazdu
    model = VehicleModel()
    car = VehicleSprite(model, start_pos=(400, 300))

    running = True
    while running:
        dt = clock.tick(60) / 1000.0  # dt w sekundach

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        throttle_input = 0.0
        steering_input = 0.0

        if keys[pygame.K_w]: throttle_input = 1.0
        if keys[pygame.K_s]: throttle_input = -1.0
        if keys[pygame.K_a]: steering_input = 1.0
        if keys[pygame.K_d]: steering_input = -1.0
        if keys[pygame.K_SPACE]: car.stop()  # Zatrzymuje wszystko
        if keys[pygame.K_r]: car.respawn() # Respawn na środku
        

        car.apply_control_input(throttle_input, steering_input, dt)
        car.update(dt)

        # Rysowanie
        screen.fill((255, 255, 255))
        car.draw(screen)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()