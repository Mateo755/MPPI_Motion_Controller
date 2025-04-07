import pygame 

from vehicle_model import VehicleModelKinematic
from vehicle_sprite import VehicleSprite


# --- MAIN ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()

    model = VehicleModelKinematic()
    car = VehicleSprite(model, start_pos=(400, 300))
    all_sprites = pygame.sprite.Group(car)

    running = True
    while running:
        dt = clock.tick(60) / 1000.0  # w sekundach

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        throttle = 0
        steering = 0

        #car.set_control(throttle, steering)
        all_sprites.update(dt)

        screen.fill((255, 255, 255))
        all_sprites.draw(screen)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
