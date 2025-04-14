import pygame
import numpy as np

# --- MODEL DYNAMIKI ---
class VehicleModel:
    def __init__(self):
        """Definicja parametrów pojazdu"""
        # ------------------Parametry pojazdu, korpusu------------------
        #   
        self.lF = 0.9               # odległość od środka masy do osi przedniej
        self.lR = 0.9               # odległość od środka masy do osi tylnej
        self.m = 230.0              # masa pojazdu
        self.Iz = 108.5             # moment bezwładności względem osi Z
        self.g = 9.81               # przyspieszenie grawitacyjne
        
        # ------------------Parametry opon-------------------------------
        # -Przód
        self.BF = 10.0               # sztywność opon (krzywizna)
        self.CF = 1.3           # kształt krzywej (nachylenie)
        self.DF = 1.0              # maksymalna siła boczna (amplituda)
        
        # -Tył
        self.BR = 12.0
        self.CR = 1.3
        self.DR = 1.0

        # -----------------Napęd i opory jazdy-----------------------------
        
        self.Cm = 0.3 * self.m             # współczynnik napędu (jak bardzo T wpływa na siłę napędową)
        self.Cr0 = 50            # opór toczenia (stały)
        self.Cr2 = 0.5            # opór aerodynamiczny (rosnący z prędkością kwadratowo)

        self.ptv = 0.05


    def curvature(self, s):
        return 0.0

    def tire_forces(self, x):
        _, _, _, vx, vy, r, delta, _ = x
        alpha_f = np.arctan2(vy + self.lF * r, vx) - delta
        alpha_r = np.arctan2(vy - self.lR * r, vx)

        Fz_f = self.lR / (self.lF + self.lR) * self.m * self.g
        Fz_r = self.lF / (self.lF + self.lR) * self.m * self.g

        Fy_f = Fz_f * self.DF * np.sin(self.CF * np.arctan(self.BF * alpha_f))
        Fy_r = Fz_r * self.DR * np.sin(self.CR * np.arctan(self.BR * alpha_r))

        return Fy_f, Fy_r

    def longitudinal_force(self, vx, T):
        return self.Cm * T - self.Cr0 - self.Cr2 * vx**2

    def torque_vectoring(self, vx, r, delta):
        rt = np.tan(delta) * vx / (self.lF + self.lR)
        return self.ptv * (rt - r) 

    def dynamics(self, x, u, s=0.0):
        s_pos, n, mu, vx, vy, r, delta, T = x
        ddelta, dT = u

        print(f"s_pos: {s_pos}, n: {n}, mu: {mu}, vx: {vx}, vy: {vy}, r: {r}, delta: {delta}, T: {T}")

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

        return np.array([ds, dn, dmu, dvx, dvy, dr, ddelta, dT])


class VehicleModelKinematic:
    def __init__(self):
        self.lF = 0.9  # distance from CoG to front axle
        self.lR = 0.9  # distance from CoG to rear axle

    def dynamics(self, x, u, s=0.0):
        """
        Prosty model kinematyczny rowerowy:
        x = [s, n, mu, vx, vy, r, delta, T]
        u = [delta_dot, T_dot] (tu ignorowane)
        """
        s, n, mu, vx, vy, r, delta, T = x
        delta_dot, T_dot = u
        L = self.lF + self.lR

        # Parametry ograniczeń
        max_delta = np.radians(35)      # maks. skręt w radianach
        max_throttle = 1.0              # maks. napęd
        min_throttle = -1.0 

        # Kinematyka
        ds = vx * np.cos(mu)
        dn = vx * np.sin(mu)
        dmu = vx / L * np.tan(delta)

        # Prosty model napędu
        acceleration = 5.0 * T
        dvx = acceleration

        dvy = 0.0
        dr = 0.0

        # Integracja sterowań
        new_delta = delta + delta_dot * 0.016  # zakładamy dt = 0.016 (60 FPS)
        new_T = T + T_dot * 0.016

        # OGRANICZENIA
        new_delta = np.clip(new_delta, -max_delta, max_delta)
        new_T = np.clip(new_T, min_throttle, max_throttle)

        # Zwracamy różnicę (zmianę)
        ddelta = new_delta - delta
        dT = new_T - T

        return np.array([ds, dn, dmu, dvx, dvy, dr, ddelta / 0.016, dT / 0.016])
