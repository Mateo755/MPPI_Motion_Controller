import pygame 
import numpy as np

from vehicle_model import VehicleModel
from vehicle_sprite import VehicleSprite

def draw_track(screen, track, color, origin_x, origin_y, scale):
    for point in track:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color, (x, y), 2)


SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900

# --- Generowanie półokręgu jako centerline ---
num_points = 100
radius = 10.0
theta = np.linspace(0, np.pi, num_points)  # od 0 do 180°
centerline = np.column_stack((
    radius * np.cos(theta),     # x = r*cos(θ)
    radius * np.sin(theta)      # y = r*sin(θ)
))

# Punkt początkowy na centerline
start_world = centerline[0]
dir_vec = centerline[1] - centerline[0]
angle = np.arctan2(dir_vec[1], dir_vec[0]) # orientacja pojazdu (radiany)


# Automatyczne wyśrodkowanie i przeskalowanie
track_center = centerline.mean(axis=0)  # [x̄, ȳ]
scale = 20.0                 # można zmieniać np. 3.0–10.0 w zależności od rozmiaru toru
origin_x = SCREEN_WIDTH // 2 - track_center[0] * scale
origin_y = SCREEN_HEIGHT // 2 - track_center[1] * scale


# --- MAIN ---
def main():
    """Uruchamia główną pętlę programu w pygame"""
    
    pygame.init()                                       # Inicjalizacja Pygame
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()                         # Timer do kontroli liczby klatek na sekundę

    model = VehicleModel()                              # Tworzy model fizyczny pojazdu
    # Punkt początkowy na centerline
    start_world = centerline[0]

    start_px = int(origin_x + start_world[0] * scale)
    start_py = int(origin_y + start_world[1] * scale)
    car = VehicleSprite(model, start_pos=(start_px, start_py), origin_x=origin_x, origin_y=origin_y, scale=scale)
    #car = VehicleSprite(model, start_pos=(400, 300))    # Tworzy obiekt graficzny pojazdu, osadzony na ekranie
    
    
    # Poprawne ustawienie stanu fizycznego pojazdu
    car.state[0] = start_world[0]
    car.state[1] = start_world[1]
    car.state[2] = angle
    
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
        draw_track(screen, centerline, (0, 0, 255), origin_x, origin_y, scale)

        car.draw(screen)                                # Rysuje pojazd (korpus + koła)
        pygame.display.flip()                           # Wyświetla nową klatkę
    #----------------------------------------------------------------------------------------------------------
    
    pygame.quit() # Kończy działanie Pygame po wyjściu z pętli


if __name__ == "__main__":
    main()
