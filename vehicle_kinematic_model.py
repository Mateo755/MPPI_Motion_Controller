import pygame
import numpy as np

# --- MODEL DYNAMIKI ---
class VehicleKinematicModel:
    """Klasa modelująca kinematykę pojazdu"""

    def __init__(self):
        self.lF = 0.9                                   # odległość od środka masy do osi przedniej
        self.lR = 0.9                                   # odległość od środka masy do osi tylnej
        self.wheel_base = self.lF + self.lR             # rozstaw osi
        self.max_steer_abs = 0.2036                     # maksymalny kąt skrętu (30°)
        self.max_accel_abs = 5.0                        # maksymalne przyspieszenie
        self.max_velocity = 30.0                        # maksymalna prędkość
        self.state = np.array([0.0, 0.0, 0.0, 15.0])    # wektor stanu -> x, y, yaw, v
        self.control = np.array([0.0, 0.0])

    def _compute_state_derivative(self, state, control):
        """
        Oblicza pochodną stanu pojazdu (model kinematyczny).
        state: [x, y, yaw, v] - pozycja, pozycja, orientacja, prędkość
        control: [steering, acceleration] - sterowanie kątem i przyspieszeniem
        """
        steer, accel = control

        # Ograniczamy wartości sterowania do fizycznych granic    
        steer = np.clip(steer, -self.max_steer_abs, self.max_steer_abs)
        accel = np.clip(accel, -self.max_accel_abs, self.max_accel_abs)
       
        x_pos, y_pos, yaw, v = state
        
        # Obliczamy pochodne stanu
        dx = v * np.cos(yaw)                            # zmiana pozycji x (prędkość w kierunku osi x)
        dy = v * np.sin(yaw)                            # zmiana pozycji y (prędkość w kierunku osi y)      
        dyaw = v / self.wheel_base * np.tan(steer)      # zmiana kąta (skręt zależny od prędkości i skrętu)    
        dv = accel                                      # zmiana prędkości (przyspieszenie)     
        
        return np.array([dx, dy, dyaw, dv])


    def next_state(self, x, u, dt):
        """
        Oblicza nowy stan pojazdu używając metody Rungego-Kutty 4. rzędu (RK4).
        
        Argumenty:
            x: aktualny stan [x, y, yaw, v]
            u: sterowanie [steering, acceleration]
            dt: krok czasowy [s]

        Zwraca:
            Nowy stan pojazdu po czasie dt
        """
        
        # RK4: cztery próbki pochodnej stanu
        # Wykonanie czterech próbnych pomiarów pośrednich w jednym kroku czasowym
        k1 = self._compute_state_derivative(x, u)
        k2 = self._compute_state_derivative(x + 0.5 * dt * k1, u)
        k3 = self._compute_state_derivative(x + 0.5 * dt * k2, u)
        k4 = self._compute_state_derivative(x + dt * k3, u)

        # Uśrednienie ważonych próbek i aktualizacja stanu 
        new_state = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        # Zwraca nowy stan pojazdu 
        return new_state

    
    def set_control(self, throttle, steering):
        """Ustawienie wektora sterowań"""
        self.control = np.array([steering, throttle])

