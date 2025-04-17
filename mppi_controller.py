import numpy as np

class MppiController:
    """Klasa implementująca sterownik bazujący na metodzie MPPI"""

    def __init__(self, model, ref_path, left_edge=None, right_edge=None, N=10, K=30, lambda_=0.5, dt=0.1, noise_sigma=(0.2, 0.7)):
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

        self.udim = 2                                   # [steer, accel]
        self.nominal_u = np.zeros((N, self.udim))       # bieżąca trajektoria sterowania
        self.ref_path = ref_path

        
        self.left_edge = left_edge
        self.right_edge = right_edge
                                                        # for visualization
        self.last_rollouts = []
        self.last_nominal = None

    def simulate_trajectory(self, x0, U):
        """Symuluje trajektorię dla danego ciągu sterowań"""

        x = np.copy(x0)
        trajectory = [x.copy()]             # dodaje stan początkowy x0 do trajektorii

        for u in U:

            u[0] = np.clip(u[0], -self.model.max_steer_abs, self.model.max_steer_abs)
            u[1] = np.clip(u[1], -self.model.max_accel_abs , self.model.max_accel_abs)
            x = self.model.next_state(x, u, self.dt)

            trajectory.append(x.copy())
        return np.array(trajectory)
    
    def compute_cost(self, trajectory):
        total_cost = 0.0
        progress = 0.0

        for state in trajectory:
            x, y, yaw, v = state

            # znajdź najbliższy punkt na ref_path
            distances = np.linalg.norm(self.ref_path - np.array([x, y]), axis=1)
            nearest_idx = np.argmin(distances)

            # SPRAWDZANIE WYJAZDU POZA TOR
            left = self.left_edge[nearest_idx]
            right = self.right_edge[nearest_idx]

            edge_vec = left - right
            point_vec = np.array([x, y]) - right
            proj = np.dot(point_vec, edge_vec) / np.dot(edge_vec, edge_vec)

            # jeśli pojazd poza "pasem jezdni"
            if proj < 0.0 or proj > 1.0:
                total_cost += 1e5  # kara za wypadnięcie z toru

            # odległość od środka toru (opcjonalnie – dla stabilności)
            min_dist = np.linalg.norm(self.ref_path[nearest_idx] - np.array([x, y]))

            # różnica orientacji względem trajektorii
            lookahead_idx = min(nearest_idx + 3, len(self.ref_path) - 2)
            target_point = self.ref_path[lookahead_idx]
            next_point = self.ref_path[lookahead_idx + 1]

            # Wektor toru (kierunek ruchu wzdłuż toru)
            tangent_vec = next_point - target_point
            tangent_vec /= np.linalg.norm(tangent_vec)

            # Wektor ruchu pojazdu (obecna prędkość i orientacja)
            vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])

            # Skalarna projekcja ruchu na tor
            delta = np.dot(vehicle_direction, tangent_vec) * v * self.dt

            # Suma postępu
            progress += delta

            path_direction = np.arctan2(next_point[1] - target_point[1],
                                        next_point[0] - target_point[0])
            yaw_diff = np.arctan2(np.sin(yaw - path_direction), np.cos(yaw - path_direction))


            # Odejmij postęp (nagroda za jazdę)
            total_cost -= 0.5 * progress  # ← większy postęp = mniejszy koszt
        return total_cost



   


    def control(self, x0):
        """Zwraca najlepsze sterowanie na podstawie aktualnego stanu"""
        
        # Perturbacje sterowania (δu) - Każdy rollout dostaje unikalny zestaw szumów.
        noises = np.random.normal(0, self.noise_sigma, size=(self.K, self.N, self.udim))
        
        # Inicjalizacja wektora funkcji kosztu trajektorii
        costs = np.zeros(self.K)
        
        self.last_rollouts = []                        # lista do przechowywania trajektorii
        
        #Monte Carlo rollouts
        for k in range(self.K):
            u_k = self.nominal_u + noises[k]
            trajectory = self.simulate_trajectory(x0, u_k)
            self.last_rollouts.append(trajectory)          # dodaj trajektorię do listy
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
        # self.last_nominal = self.simulate_trajectory(x0, self.nominal_u)

        # wybierz najlepsze rollouty
        # sorted_idxs = np.argsort(costs)
        # best_idxs = sorted_idxs[:]

        # self.last_rollouts = []
        # for idx in best_idxs:
        #     u_k = self.nominal_u + noises[idx]
        #     traj = self.simulate_trajectory(x0, u_k)
        #     self.last_rollouts.append(traj)

        #print(f"u_cmd: steer={u_cmd[0]:+.3f}, accel={u_cmd[1]:+.3f}")

        #u_cmd[1] = max(0.0, u_cmd[1]) 
        return u_cmd