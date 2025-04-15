import numpy as np

class MppiController:
    """Klasa implementująca sterownik bazujący na metodzie MPPI"""

    def __init__(self, model, ref_path, N=30, K=300, lambda_=1.0, dt=0.05, noise_sigma=(0.4, 1.0)):
        """ Inicjalizacja parametrów sterowania

        :param model: obiekt klasy VehicleModel
        :param N: długość horyzontu (w krokach)
        :param K: liczba trajektorii - rollouts
        :param lambda_: parametr eksploracji
        :param dt: krok symulacji
        :param noise_sigma: wariancje szumu sterowania (dla delta_dot i T_dot)
        """

        self.model = model
        self.N = N
        self.K = K
        self.lambda_ = lambda_
        self.dt = dt
        self.noise_sigma = np.array(noise_sigma)

        self.udim = 2                                   # [delta_dot, T_dot]
        self.nominal_u = np.zeros((N, self.udim))       # bieżąca trajektoria sterowania
        self.ref_path = ref_path

    def simulate_trajectory(self, x0, U):
        """Symuluje trajektorię dla danego ciągu sterowań"""

        x = np.copy(x0)
        trajectory = [x.copy()]             # dodaje stan początkowy x0 do trajektorii

        for u in U:
            x = self.model.next_state(x, u, self.dt)  # oblicza nowy stan na podstawie aktualnego stanu i sterowania
            trajectory.append(x.copy())
        return np.array(trajectory)
    
    def compute_cost(self, trajectory):
        total_cost = 0.0

        for state in trajectory:
            x, y, yaw, v = state

            # 1. Najbliższy punkt na trajektorii referencyjnej
            distances = np.linalg.norm(self.ref_path - np.array([x, y]), axis=1)
            nearest_idx = np.argmin(distances)
            min_dist = distances[nearest_idx]
            path_point = self.ref_path[nearest_idx]

            # 2. Kąt między orientacją pojazdu a wektorem toru
            if nearest_idx < len(self.ref_path) - 1:
                next_point = self.ref_path[nearest_idx + 1]
            else:
                next_point = path_point  # ostatni punkt, brak następnego

            path_direction = np.arctan2(next_point[1] - path_point[1],
                                        next_point[0] - path_point[0])
            yaw_diff = np.arctan2(np.sin(yaw - path_direction), np.cos(yaw - path_direction))  # wrap [-π, π]

            # 3. Prędkość – można ograniczyć lub karać przekroczenie
            v_penalty = max(0.0, v - 5.0)  # kara za prędkość powyżej 5 m/s

            # 4. Suma ważona
            dist_weight = 2.0
            angle_weight = 1.0
            speed_weight = 0.5

            cost = (dist_weight * min_dist ** 2 +
                    angle_weight * yaw_diff ** 2 +
                    speed_weight * v_penalty ** 2)

            total_cost += cost

        return total_cost


    def control(self, x0):
        """Zwraca najlepsze sterowanie na podstawie aktualnego stanu"""
        
        # Perturbacje sterowania (δu) - Każdy rollout dostaje unikalny zestaw szumów.
        noises = np.random.normal(0, self.noise_sigma, size=(self.K, self.N, self.udim))
        
        # Inicjalizacja wektora funkcji kosztu trajektorii
        costs = np.zeros(self.K)

        #Monte Carlo rollouts
        for k in range(self.K):
            u_k = self.nominal_u + noises[k]
            trajectory = self.simulate_trajectory(x0, u_k)
            costs[k] = self.compute_cost(trajectory)

        
        # Ważone poprawki do sterowania
        beta = np.min(costs)                                        # przesunięcie dla stabilności numerycznej
        weights = np.exp(-1.0 / self.lambda_ * (costs - beta))      # (costs - beta) daje najniższemu kosztowi wagę ≈ 1
        weights /= np.sum(weights + 1e-6)                           # normalizacja, by wagi dawały sumę ≈ 1
        
        delta_u =  np.sum(weights[:, None, None] * noises, axis=0)  # delta_u = sum_k w_k * δu_k  -> reward-weighted perturbations
        self.nominal_u += delta_u

        # Zastosowanie u0 i przesunięcie trajektorii
        u_cmd = self.nominal_u[0]

        self.nominal_u[:-1] = self.nominal_u[1:]    # aktualizacja: usunięcie wykorzystanego sterowania (u0) i wstawienie 0.0
        self.nominal_u[-1] = 0.0

        # Zapisz trajektorie do wizualizacji
        self.last_nominal = self.simulate_trajectory(x0, self.nominal_u)

        # wybierz najlepsze rollouty
        sorted_idxs = np.argsort(costs)
        best_idxs = sorted_idxs[:10]

        self.last_rollouts = []
        for idx in best_idxs:
            u_k = self.nominal_u + noises[idx]
            traj = self.simulate_trajectory(x0, u_k)
            self.last_rollouts.append(traj)

        return u_cmd