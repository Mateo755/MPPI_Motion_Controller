import pygame 
import numpy as np

from vehicle_sprite import VehicleSprite
from mppi_controller import MppiController


def draw_track(screen, track, color, origin_x, origin_y, scale):
    for point in track:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color, (x, y), 2)


def draw_track_edges(screen, left_edge, right_edge, origin_x, origin_y, scale, color_left=(0, 0, 0), color_right=(0, 0, 0)):
    for point in left_edge:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color_left, (x, y), 2)

    for point in right_edge:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color_right, (x, y), 2)

def draw_trajectory(screen, traj, color, scale, origin_x, origin_y, width=2, alpha=255, extend=1.5):
    if traj is None or len(traj) < 2:
        return

    for i in range(len(traj) - 1):
        x1_raw, y1_raw = traj[i][0], traj[i][1]
        x2_raw, y2_raw = traj[i+1][0], traj[i+1][1]

        # wektor od p1 do p2
        dx = x2_raw - x1_raw
        dy = y2_raw - y1_raw

        # wydłuż punkt końcowy wektora
        x2_ext = x1_raw + dx * extend
        y2_ext = y1_raw + dy * extend

        # przelicz na piksele
        x1 = int(origin_x + x1_raw * scale)
        y1 = int(origin_y + y1_raw * scale)
        x2 = int(origin_x + x2_ext * scale)
        y2 = int(origin_y + y2_ext * scale)

        pygame.draw.line(screen, color, (x1, y1), (x2, y2), width)

def draw_speedometer(screen, speed, max_speed, x=30, y=50, width=200, height=20):
        # Tło
        pygame.draw.rect(screen, (220, 220, 220), (x, y, width, height), border_radius=5)
        font = pygame.font.SysFont("monospace", 24)   

        # Kolor paska
        if speed < 2.0:
            color = (0, 180, 0)
        elif speed < 3.5:
            color = (255, 165, 0)
        else:
            color = (180, 0, 0)

        # Pasek aktualnej prędkości
        bar_width = int((speed / max_speed) * width)
        pygame.draw.rect(screen, color, (x, y, bar_width, height), border_radius=5)

        # Tekst
        text = font.render(f"Speed: {speed:.2f} m/s", True, (0, 0, 0))
        screen.blit(text, (x, y + height + 5))

def draw_steering_gauge(screen, steer_rad, max_steer_rad, x=350, y=100, radius=50):

        font_small = pygame.font.SysFont("monospace", 16)
        font_value = pygame.font.SysFont("monospace", 20)

        # Tło – łuk od 0° do 180° (czyli dół → lewa → góra → prawa → dół)
        rect = pygame.Rect(x - radius, y - radius, 2 * radius, 2 * radius)
        pygame.draw.arc(screen, (200, 200, 200), rect, np.radians(0), np.radians(180), 15)

        # Clamp steering and normalize
        steer_clamped = np.clip(steer_rad, -max_steer_rad, max_steer_rad)
        ratio = steer_clamped / max_steer_rad  # -1.0 to +1.0

        # Odwrócone: 0 rad = 270° (dół), dodatnie = w prawo
        angle = np.radians(270) + ratio * np.radians(90)

        needle_length = radius - 8
        end_x = x + needle_length * np.cos(angle)
        end_y = y + needle_length * np.sin(angle)

        pygame.draw.line(screen, (0, 0, 0), (x, y), (int(end_x), int(end_y)), 4)

        # Tekst
        label = font_small.render("Steering Angle", True, (0, 0, 0))
        screen.blit(label, (x - 70, y - radius - 25))

        steer_deg = np.degrees(steer_rad)
        value = font_value.render(f"{steer_deg:+.2f}°", True, (0, 0, 0))
        screen.blit(value, (x - 30, y  + 20))


def draw_accel_gauge(screen, accel, max_accel, x=500, y=100, radius=50):

    font_small = pygame.font.SysFont("monospace", 16)
    font_value = pygame.font.SysFont("monospace", 20)

    # Łuk tła (półkole od lewej do prawej strony)
    rect = pygame.Rect(x - radius, y - radius, 2 * radius, 2 * radius)
    pygame.draw.arc(screen, (200, 200, 200), rect, np.radians(0), np.radians(180), 15)

    # Przycinanie i normalizacja
    accel_clamped = np.clip(accel, -max_accel, max_accel)
    ratio = accel_clamped / max_accel  # -1.0 ... +1.0

    # Kąt wskazówki (0 = pionowo w dół = 270°)
    angle = np.radians(270) + ratio * np.radians(90)

    needle_length = radius - 8
    end_x = x + needle_length * np.cos(angle)
    end_y = y + needle_length * np.sin(angle)

    pygame.draw.line(screen, (0, 0, 0), (x, y), (int(end_x), int(end_y)), 4)

    # Tekst
    label = font_small.render("Acceleration", True, (0, 0, 0))
    screen.blit(label, (x - 45, y - radius - 25))

    accel_text = font_value.render(f"{accel:+.2f} m/s²", True, (0, 0, 0))
    screen.blit(accel_text, (x - 45, y + 20))

def build_track_edges(centerline, track_width=1.5):
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




# Wczytaj trajektorię z pliku CSV
ref_path = np.genfromtxt('track_data/ovalpath.csv', delimiter=',', skip_header=1)
ref_xy = ref_path[:, 0:2]  # tylko x, y

left_edge, right_edge = build_track_edges(ref_xy, track_width=3)

ref_yaw = ref_path[:, 2]
#ref_v = ref_path[:, 3]

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
    font = pygame.font.SysFont("monospace", 24)   
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

    clock = pygame.time.Clock()                         # Timer do kontroli liczby klatek na sekundę

    #Startowa pozycja z pliku CSV
    start_world = ref_xy[0]
    start_yaw = ref_yaw[0]

    start_px = int(origin_x + start_world[0] * scale)
    start_py = int(origin_y + start_world[1] * scale)

    car = VehicleSprite(start_pos=(start_px, start_py), origin_x=origin_x, origin_y=origin_y, scale=scale)  # Tworzy pojazd
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
        velocity = car.state[3]
        #speed_text = font.render(f"v = {velocity:.2f} m/s", True, (0, 0, 0))
        #print(f"v = {car.state[3]:.2f}")

        steer = car.control[0]  # albo gdzie trzymasz aktualny kąt skrętu
        alpha = 0.2  # im mniejsze, tym gładsze (np. 0.1–0.3)
        smoothed_steer = (1 - alpha) * smoothed_steer + alpha * steer

        accel = car.control[1]
        smoothed_accel = (1 - alpha) * smoothed_accel + alpha * accel
        # ==========================================
        
        screen.fill((255, 255, 255))                    # Czyści ekran (biały)

        # Rysuj tor
        draw_track(screen, ref_xy, (0, 0, 255), origin_x, origin_y, scale)
        draw_track_edges(screen, left_edge, right_edge, origin_x, origin_y, scale)


        # trajektorie MPPI
        for rollout in controller.last_rollouts[:]:
            draw_trajectory(screen, rollout, color=(150, 150, 150), scale=scale, origin_x=origin_x, origin_y=origin_y, width=3, extend=0)
        #draw_trajectory(screen, controller.last_nominal, color=(153, 0, 153), scale=scale, origin_x=origin_x, origin_y=origin_y, width=3)
                                        
        
        
        car.draw(screen) # Rysuje pojazd (korpus + koła)
        #screen.blit(speed_text, (20, 20))  # (x, y) pozycja na ekranie
        draw_speedometer(screen, velocity, max_speed=car.max_velocity)  # zakładamy maks 5 m/s
        draw_steering_gauge(screen, smoothed_steer, car.max_steer_abs)
        draw_accel_gauge(screen, smoothed_accel, car.max_accel_abs)

        
        pygame.display.flip()    # Wyświetla nową klatkę
    #----------------------------------------------------------------------------------------------------------
    
    pygame.quit() # Kończy działanie Pygame po wyjściu z pętli


if __name__ == "__main__":
    main()
