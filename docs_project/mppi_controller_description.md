# Kontroler MPPI – opis algorytmu i implementacja

## Wprowadzenie

Sterowanie MPPI (Model Predictive Path Integral) to nowoczesna metoda sterowania predykcyjnego wykorzystująca symulacje wielu możliwych trajektorii ruchu pojazdu i wybór tej, która minimalizuje zadany koszt. W odróżnieniu od klasycznych metod MPC, MPPI nie wymaga liniaryzacji modelu ani rozwiązywania równań nieliniowych – opiera się na podejściu prób losowych (sampling-based).

Metoda ta sprawdza się szczególnie dobrze w zastosowaniach symulacyjnych i w czasie rzeczywistym, ze względu na swoją prostotę obliczeniową i odporność na nieliniowości.

---

## Struktura sterownika

Sterownik MPPI wykonuje następujące kroki w każdej iteracji sterowania:

1. **Generuje K trajektorii sterowania** przez dodanie szumu do nominalnej trajektorii `u_nominal`,
2. **Symuluje K trajektorii ruchu pojazdu** za pomocą modelu kinematycznego,
3. **Oblicza koszt każdej trajektorii**, uwzględniając:
   - progres wzdłuż toru,
   - odchylenie od kierunku trajektorii (`yaw_diff`),
   - wyjazd poza tor (`is_inside_track`),
4. **Wybiera ważoną średnią poprawkę** do sterowania na podstawie kosztów (softmin),
5. **Aktualizuje nominalną trajektorię** i wykonuje pierwsze sterowanie `u0`.

---

## Wektor sterowania

Każde sterowanie w MPPI to wektor:

```
u = [steer, accel]
```

czyli:
- `steer` – kąt skrętu (w radianach),
- `accel` – przyspieszenie (m/s²).

---

## Parametry algorytmu

| Parametr     | Znaczenie                                              |
|--------------|--------------------------------------------------------|
| `N`          | Horyzont planowania (liczba kroków symulacji)         |
| `K`          | Liczba trajektorii próbnych (Monte Carlo rollouts)    |
| `lambda_`    | Parametr eksploracji (im mniejszy, tym bardziej wybierane są trajektorie o niskim koszcie) |
| `dt`         | Krok czasowy symulacji                                 |
| `noise_sigma`| Wariancje szumu dodawanego do sterowania              |

---

## Funkcja kosztu

Koszt trajektorii uwzględnia:
- postęp wzdłuż toru (ujemny koszt),
- wyjazd poza tor (kara +3.0),
- różnicę między orientacją pojazdu a kierunkiem trajektorii (`yaw_diff²`),
- dostosowanie do krzywizny zakrętów (większa kara w zakrętach).

---

## Implementacja

Sterownik zaimplementowano jako klasę `MppiController`, zawierającą m.in. metody:

- `simulate_trajectory(x0, U)` – zwraca trajektorię stanu dla sekwencji sterowań,
- `compute_cost(trajectory)` – liczy koszt trajektorii,
- `control(x0)` – główna funkcja zwracająca sterowanie do wykonania.

---

## Wymagania i ograniczenia

MPPI wymaga:
- modelu symulacyjnego `next_state(...)`,
- toru referencyjnego (`ref_path`) oraz opcjonalnie krawędzi toru (`left_edge`, `right_edge`),
- stałego czasu próbkowania.

Ograniczenia:
- metoda próbna – jakość zależy od liczby trajektorii `K`,
- brak gwarancji optymalności, ale dobre właściwości praktyczne.

---

## Zastosowanie

MPPI nadaje się do zastosowań w robotyce, pojazdach autonomicznych i systemach, w których:
- model jest nieliniowy,
- działanie musi być realizowane w czasie rzeczywistym,
- dopuszczalna jest stochastyczna optymalizacja.