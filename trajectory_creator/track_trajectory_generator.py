import numpy as np

def generate_loop_track(radius=60, width=7, num_points=1000):
    """Generuje współrzędne punktów toru i środkowej linii trajektorii"""
    t = np.linspace(0, 2 * np.pi, num_points)

    # Złożona środkowa trajektoria: suma sinusów
    # x_center = radius * np.sin(t) - 1 * np.sin(6*t) - 4 * np.sin(4*t) 
    # y_center = radius * np.cos(t) + 1 * np.cos(4*t) + 7 *  np.cos(6*t) - 1 * np.cos(8*t)

    x_center = radius * np.sin(t) + 8 * np.sin(3*t) + 4 * np.sin(5*t)
    y_center = radius * np.cos(t) - 6 * np.cos(4*t)

    dx = np.gradient(x_center)
    dy = np.gradient(y_center)
    length = np.sqrt(dx**2 + dy**2)
    nx = -dy / length
    ny = dx / length

    x_left = x_center + width * nx
    y_left = y_center + width * ny
    x_right = x_center - width * nx
    y_right = y_center - width * ny

    center_line = list(zip(x_center, y_center))
    left_edge = list(zip(x_left, y_left))
    right_edge = list(zip(x_right, y_right))
    
    return center_line, left_edge, right_edge