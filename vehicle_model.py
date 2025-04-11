import pygame
import numpy as np

# --- MODEL DYNAMIKI ---
class VehicleModel:
    """Klasa modelująca dynamikę pojazdu"""

    def __init__(self):
        """Definicja parametrów pojazdu"""
        # ------------------Parametry pojazdu, korpusu------------------
        #   
        self.lF = 0.9               # odległość od środka masy do osi przedniej
        self.lR = 0.9               # odległość od środka masy do osi tylnej
        self.m = 200.0              # masa pojazdu
        self.Iz = 100.0             # moment bezwładności względem osi Z
        self.g = 9.81               # przyspieszenie grawitacyjne
        
        # ------------------Parametry opon-------------------------------
        # -Przód
        self.BF = 5.0               # sztywność opon (krzywizna)
        self.CF = 1.3               # kształt krzywej (nachylenie)
        self.DF = 1.0               # maksymalna siła boczna (amplituda)
        
        # -Tył
        self.BR = 5.0
        self.CR = 1.3
        self.DR = 1.0

        # -----------------Napęd i opory jazdy-----------------------------
        
        self.Cm = 400.0             # współczynnik napędu (jak bardzo T wpływa na siłę napędową)
        self.Cr0 = 50.0          # opór toczenia (stały)
        self.Cr2 = 1.0              # opór aerodynamiczny (rosnący z prędkością kwadratowo)

        self.ptv = 10.0

    def curvature(self, s):
        """Zwraca krzywiznę toru, czyli jak mocno zakręca droga w miejscu s"""
        return 0.0

    def tire_forces(self, x):
        """Oblicza siły boczne opon przednich i tylnych - uproszczonego modelu Pacejki"""
        _, _, _, vx, vy, r, delta, _ = x
        
        
        # -------------------Kąty uślizgu opon przednich i tylnych---------------------
        alpha_f = np.arctan2(vy + self.lF * r, vx) - delta          
        alpha_r = np.arctan2(vy - self.lR * r, vx)

        # -------------------Pionowe siły nacisku (siła ciężaru na oś)-----------------
        # rozdzielają ciężar między przód i tył (proporcjonalnie do lF, lR)
        
        Fn_f = self.lR / (self.lF + self.lR) * self.m * self.g
        Fn_r = self.lF / (self.lF + self.lR) * self.m * self.g

        # ----------Rzeczywiste siły boczne (hamujące drift lub powodujące skręt)-------

        Fy_f = Fn_f * self.DF * np.sin(self.CF * np.arctan(self.BF * alpha_f))
        Fy_r = Fn_r * self.DR * np.sin(self.CR * np.arctan(self.BR * alpha_r))

        return Fy_f, Fy_r

    def longitudinal_force(self, vx, T):
        """Oblicza siłę wzdłużną (napędzającą) pojazd"""
        return self.Cm * T - self.Cr0 - self.Cr2 * vx**2

    def torque_vectoring(self, vx, r, delta):
        """Oblicza dodatkowy moment korekcyjny, który stabilizuje tor jazdy"""
        rt = np.tan(delta) * vx / (self.lF + self.lR)  # teoretyczna prędkość kątowa pojazdu przy danym skręcie kół (zakładana)
        return self.ptv * (rt - r)                     # (rt - r) różnica, czyli błąd rotacji

    def dynamics(self, x, u, s=0.0):
        """Generuje pochodną wektora stanu,oblicza następny krok symulacji"""
       
        # ------------------------Wektor Stanu----------------------------
        
        # s_pos	- pozycja wzdłuż toru (krzywoliniowy postęp)	
        # n	    - odległość boczna od środka toru (lateral error)	
        # mu	- orientacja pojazdu względem toru
        # vx	- prędkość wzdłużna (do przodu)	
        # vy	- prędkość boczna (do boku)
        # r	    - prędkość kątowa (yaw rate)	
        # delta	- kąt skrętu przednich kół	
        # T   	- napęd pojazdu

        s_pos, n, mu, vx, vy, r, delta, T = x

        #------------------------Wektor sterowań-------------------------
        
        #ddelta: prędkość zmiany skrętu (delta-dot)
        #dT: prędkość zmiany napędu (T-dot)
        
        ddelta, dT = u

        #---------------------------------------------------------------

        kappa = self.curvature(s_pos)
        Fy_f, Fy_r = self.tire_forces(x)
        Fx = self.longitudinal_force(vx, T)
        Mtv = self.torque_vectoring(vx, r, delta)

        ds = (vx * np.cos(mu) - vy * np.sin(mu)) / (1 - n * kappa)
        dn = vx * np.sin(mu) + vy * np.cos(mu)
        dmu = r - kappa * ds
        dvx = (Fx - Fy_f * np.sin(delta) + self.m * vy * r) / self.m
        dvy = (Fy_r + Fy_f * np.cos(delta) - self.m * vx * r) / self.m
        dr = (Fy_f * self.lF * np.cos(delta) - Fy_r * self.lR + Mtv) / self.Iz

        return np.array([ds, dn, dmu, dvx, dvy, dr, ddelta, dT])  # pochodna wektora stanu



