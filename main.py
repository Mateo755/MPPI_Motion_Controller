import pygame 
import numpy as np

from vehicle_model import VehicleModel
from vehicle_sprite import VehicleSprite

SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900

# Wczytaj punkty toru z CSV
centerline = np.loadtxt("track_data/centerline.csv", delimiter=",", skiprows=1)
left_edge = np.loadtxt("track_data/left_edge.csv", delimiter=",", skiprows=1)
right_edge = np.loadtxt("track_data/right_edge.csv", delimiter=",", skiprows=1)

# Automatyczne wyśrodkowanie i przeskalowanie
track_center = centerline.mean(axis=0)  # [x̄, ȳ]
scale = 5.0  # można zmieniać np. 3.0–10.0 w zależności od rozmiaru toru
origin_x = SCREEN_WIDTH // 2 - track_center[0] * scale
origin_y = SCREEN_HEIGHT // 2 - track_center[1] * scale

def draw_track(screen, track, color, origin_x, origin_y, scale):
    for point in track:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color, (x, y), 2)


# --- MAIN ---
def main():
    """Uruchamia główną pętlę programu w pygame"""
    
    pygame.init()                                       # Inicjalizacja Pygame
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()                         # Timer do kontroli liczby klatek na sekundę

    model = VehicleModel()                              # Tworzy model fizyczny pojazdu
    car = VehicleSprite(model, start_pos=(400, 300))    # Tworzy obiekt graficzny pojazdu, osadzony na ekranie
    
    running = True
    # -------------------------------------------------------------------------------------------------------
    while running:
        dt = clock.tick(60) / 1000.0                    # Odlicza czas od ostatniej klatki (w sekundach)
        #===========================================

        # Obsługa zdarzeń
        for event in pygame.event.get():                
            if event.type == pygame.QUIT:
                running = False
        # ==========================================
        
        keys = pygame.key.get_pressed()
        throttle = 0
        steering = 0

        #car.set_control(throttle, steering)            # Ustawienie sterowania
        car.update(dt)                                  # Liczy nową pozycję, prędkości itd. na podstawie modelu
        # ==========================================
        
        screen.fill((255, 255, 255))                    # Czyści ekran (biały)

        # Rysuj tor
        draw_track(screen, left_edge, (255, 0, 0), origin_x, origin_y, scale)
        draw_track(screen, right_edge, (0, 255, 0), origin_x, origin_y, scale)
        draw_track(screen, centerline, (0, 0, 255), origin_x, origin_y, scale)

        car.draw(screen)                                # Rysuje pojazd (korpus + koła)
        pygame.display.flip()                           # Wyświetla nową klatkę
    #----------------------------------------------------------------------------------------------------------
    
    pygame.quit() # Kończy działanie Pygame po wyjściu z pętli


if __name__ == "__main__":
    main()
