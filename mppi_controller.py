import numpy as np

class MppiController:
    """Klasa implementująca sterownik bazujący na metodzie MPPI"""

    def __init__(self, model, ref_path, left_edge=None, right_edge=None, N=10, K=10, lambda_=0.4, dt=0.1, noise_sigma=(0.2, 0.6)):
        """
        Inicjalizuje kontroler MPPI wraz z parametrami planowania

        Args:
            model: Model pojazdu z metodą next_state()
            ref_path: Ścieżka referencyjna (punkty środkowej linii toru)
            left_edge: Punkty lewej krawędzi toru (opcjonalnie)
            right_edge: Punkty prawej krawędzi toru (opcjonalnie)
            N: Liczba kroków planowania w przyszłość, długość horyzontu 
            K: Liczba losowych trajektorii (rolloutów)
            lambda_: Parametr eksploracji (im mniejszy, tym bardziej wybierane są tylko dobre trajektorie)
            dt: Krok czasowy, krok symulacji
            noise_sigma: wariancje szumu sterowania (dla delta_dot i T_dot)
        """
        
        self.model = model
        self.N = N
        self.K = K
        self.lambda_ = lambda_
        self.dt = dt
        self.noise_sigma = np.array(noise_sigma)

        self.udim = 2                                   # [steer, accel]
        self.nominal_u = np.zeros((N, self.udim))       # Bieżąca trajektoria sterowania
        self.ref_path = ref_path
        self.track_width = 6.0                          # Szerokość toru

        
        self.left_edge = left_edge
        self.right_edge = right_edge
                                                        # For visualization
        self.last_rollouts = []                         # Ostatnio symulowane trajektorie
        self.last_nominal = None

    def simulate_trajectory(self, x0, U):
        """
        Symuluje trajektorię stanu pojazdu dla zadanej sekwencji sterowań.

        Args:
            x0: Stan początkowy pojazdu.
            U: Sekwencja sterowań (macierz Nx2).

        Returns:
            trajectory: Lista stanów pojazdu w czasie.
        """

        x = np.copy(x0)
        trajectory = [x.copy()]             # dodaje stan początkowy x0 do trajektorii

        for u in U:
            # Ograniczamy sterowania do fizycznych limitów
            u[0] = np.clip(u[0], -self.model.max_steer_abs, self.model.max_steer_abs)
            u[1] = np.clip(u[1], -self.model.max_accel_abs , self.model.max_accel_abs)
            x = self.model.next_state(x, u, self.dt)

            trajectory.append(x.copy())
        
        return np.array(trajectory)
    

    def is_inside_track(self, point):
        """
        Sprawdza, czy dany punkt znajduje się pomiędzy lewą a prawą krawędzią toru

        Args:
            point: Współrzędne punktu (x, y)

        Returns:
            True, jeśli punkt leży w obrębie toru
        """

        point = np.asarray(point)

        left = self.left_edge
        right = self.right_edge

        edge_vecs = left - right
        point_vecs = point - right  # Wektory od prawej krawędzi do punktu
        
        edge_len2 = np.einsum('ij,ij->i', edge_vecs, edge_vecs)
        proj = np.einsum('ij,ij->i', point_vecs, edge_vecs) / (edge_len2 + 1e-8)

        mask = (proj >= 0.0) & (proj <= 1.0)
        if not np.any(mask):
            return False

        proj_points = right + proj[:, None] * edge_vecs
        dists = np.linalg.norm(proj_points - point, axis=1)

        return np.any(dists[mask] < self.track_width / 2.0)

    
    
    def compute_cost(self, trajectory):
        """
        Oblicza koszt trajektorii na podstawie zgodności z torem i postępu wzdłuż ścieżki

        Args:
            trajectory: Lista stanów [x, y, yaw, v]

        Returns:
            Łączny koszt (float) trajektorii
        """
                
        total_cost = 0.0
        progress = 0.0

        for state in trajectory:
            x, y, yaw, v = state
            point = np.array([x, y])

            # Sprawdź, czy pojazd znajduje się w torze
            inside = self.is_inside_track(point)
            # Kara za wyjazd poza tor
            if not inside:
                total_cost += 3.0
                
            # najbliższy punkt na torze(lookahead)
            distances = np.linalg.norm(self.ref_path - point, axis=1)
            nearest_idx = np.argmin(distances)
            lookahead_idx = min(nearest_idx + 3, len(self.ref_path) - 2)
            
             # Kierunki zakrętu
            p1 = self.ref_path[lookahead_idx]
            p2 = self.ref_path[lookahead_idx + 1]
            p3 = self.ref_path[min(lookahead_idx + 2, len(self.ref_path) - 1)]

            # Na podstawie trzech punktów odniesienia (lookahead) obliczana jest lokalna krzywizna toru
            d1 = p2 - p1
            d2 = p3 - p2
            angle_diff = np.arctan2(np.cross(d1, d2), np.dot(d1, d2))  # skręt toru
            curvature = abs(angle_diff)  # im większy, tym ostrzejszy zakręt

            # Nagroda za postęp w zależności od krzywizny toru
            # Dynamiczne ważenie: na prostych duża nagroda, w zakrętach mała
            speed_reward_weight = np.clip(1.0 - curvature * 5.0, 0.5, 0.8)

 
            # Różnica orientacji względem trajektorii
            lookahead_idx = min(nearest_idx + 3, len(self.ref_path) - 2)
            target_point = self.ref_path[lookahead_idx]
            next_point = self.ref_path[lookahead_idx + 1]
            min_dist = np.linalg.norm(target_point - np.array([x, y]))
 
            # Wektor toru (kierunek ruchu wzdłuż toru)
            tangent_vec = next_point - target_point
            tangent_vec /= np.linalg.norm(tangent_vec)
 
            # Wektor ruchu pojazdu (obecna prędkość i orientacja)
            vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])
            # Skalarna projekcja ruchu na tor
            # Obliczana jest skalarna projekcja wektora prędkości pojazdu na kierunek toru, co przekłada się na rzeczywisty postęp wzdłuż toru
            delta = np.dot(vehicle_direction, tangent_vec) * v * self.dt
            progress += delta   

            # Odchylenie kierunku jazdy od trajektorii
            path_direction = np.arctan2(next_point[1] - target_point[1],
                                        next_point[0] - target_point[0])  

            # Różnica orientacji pojazdu względem lokalnego kierunku toru (tzw. yaw error)
            yaw_diff = np.arctan2(np.sin(yaw - path_direction), np.cos(yaw - path_direction))
            
            # Koszt zostaje pomniejszony o nagrodę za postęp, ważoną przez warunki toru (np. prosty odcinek lub zakręt).
            total_cost -= speed_reward_weight * progress
           
            # Im większa prędkość i błąd orientacji, tym większy koszt
            total_cost +=  v * yaw_diff ** 2

        return total_cost





    def control(self, x0):
        """
        Główna funkcja sterująca – wybiera najlepsze sterowanie na podstawie MPPI i aktualnego stanu

        Args:
            x0: Aktualny stan pojazdu

        Returns:
            u_cmd: Optymalne sterowanie (skręt, przyspieszenie) do wykonania teraz
        """
        
        # Generowany szum dla K trajektorii (rolloutów)
        noises = np.random.normal(0, self.noise_sigma, size=(self.K, self.N, self.udim))
        
        # Inicjalizacja wektora funkcji kosztu trajektorii
        costs = np.zeros(self.K)
        
        # Lista do przechowywania trajektorii
        self.last_rollouts = []  
        
        # Symulowanie trajektorii z losowymi szumami
        # Monte Carlo rollouts
        for k in range(self.K):
            u_k = self.nominal_u + noises[k]
            trajectory = self.simulate_trajectory(x0, u_k)
            self.last_rollouts.append(trajectory)          # Dodaj trajektorię do listy
            costs[k] = self.compute_cost(trajectory)

        
        # Ważone poprawki do sterowania
        # Przesunięcie dla stabilności numerycznej
        # Funkcja exp(-cost) bardzo szybko zmierza do zera, gdy cost jest duży. 
        # Wówczas może dojść do underflow czyli zaokrąglenia do zera 
        beta = np.min(costs)                                      
        # Zmniejszamy zakres wartości przekazywanych do exp() co stabilizuje obliczenia.

        # Wyliczenie wag proporconalnych do kosztów
        # Im mniejszy koszt, tym większa waga
        weights = np.exp(-1.0 / self.lambda_ * (costs - beta))        # funkcja boltzmanna exp(-cost / lambda)
        
        # dodanie 1e-6 żeby zapobiec dzieleniu przez zero w skrajnych przypadkach
        weights /= np.sum(weights + 1e-6)                           
        
        # Normalizacja wagi sumują się do 1.0
        
        # Wagi mówią, jak bardzo każda perturbacja jest „ważna” przy uśrednianiu
        # delta_u - Wynik uśrednienia (ważonego) perturbacji
        delta_u =  np.sum(weights[:, None, None] * noises, axis=0)  
        
        # Aktualna sekwencja sterowania - aktualizacja o najlepszą perturbację
        self.nominal_u += delta_u

        # MPPI planuje całą trajektorię sterowania na N kroków do przodu, ale w każdej iteracji wykonuje tylko pierwszy krok
        # Wybieramy tylko pierwsze sterowanie z trajektorii
        u_cmd = self.nominal_u[0]


        # Przesunięcie trajektorii sterowania w lewo, bo próbka sterowania u0 została już wykorzystana
        # Aktualizacja: usunięcie wykorzystanego sterowania (u0) i wstawienie 0.0
        self.nominal_u[:-1] = self.nominal_u[1:]   
        self.nominal_u[-1] = 0.0


        return u_cmd  # sterowanie do wykonania w tym kroku