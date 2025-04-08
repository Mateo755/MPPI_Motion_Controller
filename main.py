import pygame 

from vehicle_model import VehicleModel
from vehicle_sprite import VehicleSprite


# --- MAIN ---
def main():
    """Uruchamia główną pętlę programu w pygame"""
    
    pygame.init()                                       # Inicjalizacja Pygame
    screen = pygame.display.set_mode((800, 600))
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
        car.draw(screen)                                # Rysuje pojazd (korpus + koła)
        pygame.display.flip()                           # Wyświetla nową klatkę
    #----------------------------------------------------------------------------------------------------------
    
    pygame.quit() # Kończy działanie Pygame po wyjściu z pętli


if __name__ == "__main__":
    main()
