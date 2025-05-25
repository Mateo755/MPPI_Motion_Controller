import pygame
import numpy as np

def draw_track(screen, track, color, origin_x, origin_y, scale):
    """Rysuje środkową linię toru na podstawie listy punktów."""

    for point in track:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color, (x, y), 2)


def draw_track_edges(screen, left_edge, right_edge, origin_x, origin_y, scale, color_left=(0, 0, 0), color_right=(0, 0, 0)):
    """Rysuje lewą i prawą krawędź toru na podstawie punktów granicznych."""

    
    for point in left_edge:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color_left, (x, y), 2)

    for point in right_edge:
        x = int(origin_x + point[0] * scale)
        y = int(origin_y + point[1] * scale)
        pygame.draw.circle(screen, color_right, (x, y), 2)


def draw_trajectory(screen, traj, color, scale, origin_x, origin_y, width=2, alpha=255, extend=1.5):
    """Rysuje przewidywaną trajektorię pojazdu jako linię pomiędzy kolejnymi punktami."""

    
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
        """Wyświetla poziomy wskaźnik aktualnej prędkości pojazdu."""

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
        """Wyświetla półokrągły wskaźnik kąta skrętu kierownicy."""

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
    """Wyświetla półokrągły wskaźnik przyspieszenia lub hamowania pojazdu."""

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
