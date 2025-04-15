import pygame 
import numpy as np

from vehicle_sprite import VehicleSprite
from mppi_controller import MppiController


def draw_track(screen, track, color, origin_x, origin_y, scale):
    for point in track:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color, (x, y), 2)

def draw_trajectory(screen, traj, color, scale, origin_x, origin_y, width=1, alpha=255):
    if traj is None:
        return
    for i in range(len(traj)-1):
        x1 = int(origin_x + traj[i][0] * scale)
        y1 = int(origin_y + traj[i][1] * scale)
        x2 = int(origin_x + traj[i+1][0] * scale)
        y2 = int(origin_y + traj[i+1][1] * scale)
        pygame.draw.line(screen, color, (x1, y1), (x2, y2), width)


# Wczytaj trajektorię z pliku CSV
ref_path = np.genfromtxt('track_data/ovalpath.csv', delimiter=',', skip_header=1)
ref_xy = ref_path[:, 0:2]  # tylko x, y
ref_yaw = ref_path[:, 2]
ref_v = ref_path[:, 3]

SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900


#2. Automatyczne wyśrodkowanie i przeskalowanie
track_center = ref_xy.mean(axis=0)
scale = 15.0
origin_x = SCREEN_WIDTH // 2 - track_center[0] * scale
origin_y = SCREEN_HEIGHT // 2 - track_center[1] * scale


# --- MAIN ---
def main():
    """Uruchamia główną pętlę programu w pygame"""
    
    pygame.init()                                       # Inicjalizacja Pygame
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

    clock = pygame.time.Clock()                         # Timer do kontroli liczby klatek na sekundę

    #Startowa pozycja z pliku CSV
    start_world = ref_xy[0]
    start_yaw = ref_yaw[0]

    start_px = int(origin_x + start_world[0] * scale)
    start_py = int(origin_y + start_world[1] * scale)

    car = VehicleSprite(start_pos=(start_px, start_py), origin_x=origin_x, origin_y=origin_y, scale=scale)  # Tworzy pojazd
    controller = MppiController(model=car, ref_path=ref_xy)
     
    # Poprawne ustawienie stanu fizycznego pojazdu
    car.state[0] = start_world[0]
    car.state[1] = start_world[1]
    car.state[2] = start_yaw
    
    running = True
    frame_counter = 0
    # -------------------------------------------------------------------------------------------------------
    while running:
        dt = clock.tick(60) / 1000.0                    # Odlicza czas od ostatniej klatki (w sekundach)
        frame_counter += 1
        #===========================================

        # Obsługa zdarzeń
        for event in pygame.event.get():                
            if event.type == pygame.QUIT:
                running = False
        # ==========================================
        
        #throttle = 0.3
        #steering = 0.4
        #car.set_control(throttle, steering)            # Ustawienie sterowania
        
        #if frame_counter % 3 == 0:
        u = controller.control(car.state.copy())
        car.set_control(throttle=u[1], steering=u[0])
        
        
        car.update(dt)                                  # Liczy nową pozycję, prędkości itd. na podstawie modelu
        #print(f"v = {car.state[3]:.2f}")
        # ==========================================
        
        screen.fill((255, 255, 255))                    # Czyści ekran (biały)

        # Rysuj tor
        draw_track(screen, ref_xy, (0, 0, 255), origin_x, origin_y, scale)

        # trajektorie MPPI
        # for rollout in controller.last_rollouts[:10]:
        #     draw_trajectory(screen, rollout, color=(150, 150, 150), scale=scale, origin_x=origin_x, origin_y=origin_y, width=1)
        # draw_trajectory(screen, controller.last_nominal, color=(153, 0, 153), scale=scale, origin_x=origin_x, origin_y=origin_y, width=2)
                                        
        
        
        car.draw(screen) # Rysuje pojazd (korpus + koła)
        
        pygame.display.flip()    # Wyświetla nową klatkę
    #----------------------------------------------------------------------------------------------------------
    
    pygame.quit() # Kończy działanie Pygame po wyjściu z pętli


if __name__ == "__main__":
    main()
