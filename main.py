import pygame 
import numpy as np

from graphics_utils import (
    draw_track,
    draw_track_edges,
    draw_trajectory,
    draw_speedometer,
    draw_steering_gauge,
    draw_accel_gauge
)

from vehicle_sprite import VehicleSprite
from mppi_controller import MppiController


def build_track_edges(centerline, track_width=1.5):
    """Tworzy lewe i prawe krawędzie toru na podstawie punktów środkowej linii"""
    
    left_edge = []
    right_edge = []

    for i in range(len(centerline) - 1):
        p1 = centerline[i]
        p2 = centerline[i + 1]
        direction = p2 - p1
        direction /= np.linalg.norm(direction)
        normal = np.array([-direction[1], direction[0]])  # lewo

        left_edge.append(p1 + normal * track_width)
        right_edge.append(p1 - normal * track_width)

    # dla ostatniego punktu kopiujemy ostatnią orientację
    left_edge.append(left_edge[-1])
    right_edge.append(right_edge[-1])

    return np.array(left_edge), np.array(right_edge)




# Wczytanie trajektorii z pliku CSV
ref_path = np.genfromtxt('track_data/ovalpath.csv', delimiter=',', skip_header=1)

# Wyodrębnienie współrzędnych XY i obliczenie krawędzi toru
ref_xy = ref_path[:, 0:2]  # tylko x, y
left_edge, right_edge = build_track_edges(ref_xy, track_width=3)

# Odczyt orientacji 
ref_yaw = ref_path[:, 2]

SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900


# Obliczenie punktu środka i skalowania do rozmiaru okna
track_center = ref_xy.mean(axis=0)
scale = 15.0
origin_x = SCREEN_WIDTH // 2 - track_center[0] * scale
origin_y = SCREEN_HEIGHT // 2 - track_center[1] * scale


# --- MAIN ---
def main():
    """
    Uruchamia główną pętlę Pygame: inicjalizuje tor, pojazd, kontroler i wyświetla interaktywną symulację.
    """
    
    # Inicjalizacja Pygame i ekranu
    pygame.init()    
    font = pygame.font.SysFont("monospace", 24)   
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

    clock = pygame.time.Clock()  # Timer do kontroli liczby klatek na sekundę

    #Startowa pozycja z pliku CSV
    start_world = ref_xy[0]
    start_yaw = ref_yaw[0]

    start_px = int(origin_x + start_world[0] * scale)
    start_py = int(origin_y + start_world[1] * scale)

    # Inicjalizacja pojazdu i kontrolera MPPI
    car = VehicleSprite(start_pos=(start_px, start_py), origin_x=origin_x, origin_y=origin_y, scale=scale)  
    controller = MppiController(model=car, ref_path=ref_xy, left_edge=left_edge, right_edge=right_edge)
     
    # Poprawne ustawienie stanu fizycznego pojazdu
    car.state[0] = start_world[0]
    car.state[1] = start_world[1]
    car.state[2] = start_yaw
    
    running = True
    frame_counter = 0
    smoothed_steer = 0.0
    smoothed_accel = 0.0
    # -------------------------------------------------------------------------------------------------------
    # Pętla symulacji – działa do momentu zamknięcia okna
    while running:
        dt = clock.tick(60) / 1000.0     # Odlicza czas od ostatniej klatki (w sekundach)
        frame_counter += 1
        #===========================================

        # Obsługa zdarzeń
        for event in pygame.event.get():                
            if event.type == pygame.QUIT:
                running = False
        # ==========================================
        

        # Ustawienie sterowania

        #throttle = 0.3
        #steering = 0.4
        #car.set_control(throttle, steering)            

        # Obliczanie sterowania przez kontroler MPPI
        u = controller.control(car.state.copy())
        car.set_control(throttle=u[1], steering=u[0])
        
        # Aktualizacja stanu fizycznego i pozycji pojazdu
        car.update(dt)  # Liczy nową pozycję, prędkości itd. na podstawie modelu
        velocity = car.state[3]

        steer = car.control[0]  # aktualny kąt skrętu
        alpha = 0.2  # im mniejsze, tym gładsze (np. 0.1–0.3)
        smoothed_steer = (1 - alpha) * smoothed_steer + alpha * steer

        accel = car.control[1]
        smoothed_accel = (1 - alpha) * smoothed_accel + alpha * accel
        # ==========================================
        # Czyszczenie ekranu i rysowanie elementów graficznych
        
        screen.fill((255, 255, 255))  # Czyści ekran (biały)

        # Rysuje tor
        draw_track(screen, ref_xy, (0, 0, 255), origin_x, origin_y, scale)
        draw_track_edges(screen, left_edge, right_edge, origin_x, origin_y, scale)


        # Rysuje Trajektorie MPPI
        for rollout in controller.last_rollouts[:]:
            draw_trajectory(screen, rollout, color=(150, 150, 150), scale=scale, origin_x=origin_x, origin_y=origin_y, width=3, extend=0)
        #draw_trajectory(screen, controller.last_nominal, color=(153, 0, 153), scale=scale, origin_x=origin_x, origin_y=origin_y, width=3)
                                        
        
        
        car.draw(screen) # Rysuje pojazd (korpus + koła)
        draw_speedometer(screen, velocity, max_speed=car.max_velocity)  
        draw_steering_gauge(screen, smoothed_steer, car.max_steer_abs)
        draw_accel_gauge(screen, smoothed_accel, car.max_accel_abs)

        
        pygame.display.flip()    # Wyświetla nową klatkę
    #----------------------------------------------------------------------------------------------------------
    
    pygame.quit() # Kończy działanie Pygame po wyjściu z pętli


if __name__ == "__main__":
    main()
