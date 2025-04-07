import pygame
from track_trajectory_generator import generate_loop_track

# === Ustawienia ekranu ===
WIDTH, HEIGHT = 800, 800
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (50, 50, 50)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

SCALE = 5     # Ile pikseli na "metr"
OFFSET = (WIDTH // 2, HEIGHT // 2)

def world_to_screen(x, y):
    """Zamiana współrzędnych świata na ekran"""
    sx = int(x * SCALE + OFFSET[0])
    sy = int(-y * SCALE + OFFSET[1])  # Odwrócony Y (Pygame rośnie w dół)
    return (sx, sy)


def draw_path(screen, points, color, width=2):
    for i in range(len(points) - 1):
        p1 = world_to_screen(*points[i])
        p2 = world_to_screen(*points[i + 1])
        pygame.draw.line(screen, color, p1, p2, width)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Tor wyścigowy")
    clock = pygame.time.Clock()

    # Generuj tor
    center_line, left_edge, right_edge = generate_loop_track()

    running = True
    while running:
        screen.fill(WHITE)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Rysuj tor
        draw_path(screen, left_edge, BLACK, 3)
        draw_path(screen, right_edge, BLACK, 3)
        draw_path(screen, center_line, BLUE, 1)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
