
# Ogólne informacje o Model Predictive Path Integral (MPPI)

## 1. Wprowadzenie

Model Predictive Path Integral (MPPI) to nowoczesna metoda sterowania należąca do rodziny sterowań predykcyjnych (MPC), wykorzystująca stochastyczne próbkowanie trajektorii i rachunek wariacyjny oparty na całkach po ścieżkach. W odróżnieniu od klasycznego MPC, MPPI nie wymaga obliczania gradientów ani analitycznego modelu układu, a jedynie możliwości symulacji trajektorii.

## 2. Zasada działania MPPI

### 2.1 Ogólna idea

MPPI dąży do wyznaczenia optymalnej sekwencji sterowań, minimalizującej zadaną funkcję kosztu na horyzoncie czasowym. Wykorzystuje przy tym próbki (trajektorie) generowane przez losowe perturbacje nominalnych sterowań, a następnie oblicza ważoną średnią tych perturbacji z wagami opartymi na kosztach trajektorii.

### 2.2 Etapy algorytmu

1. **Inicjalizacja**: ustalenie nominalnej trajektorii sterowań.
2. **Generowanie trajektorii**: losowe perturbacje sterowań na horyzoncie czasowym.
3. **Symulacja**: każda trajektoria jest symulowana przez model dynamiczny układu.
4. **Obliczenie kosztu**: koszt całkowity każdej trajektorii.
5. **Ważenie**: zastosowanie reguły ważenia softmax.
6. **Aktualizacja sterowania**: nowa trajektoria sterowań to uśredniona suma perturbacji ważona przez `w_i`.

## 3. Wzory matematyczne i ich znaczenie

### 3.1 Funkcja kosztu

$$J = \sum_{t=0}^{T-1} \ell(x_t, u_t) + \Phi(x_T)$$


**Opis symboli:**
- $J$ – całkowity koszt trajektorii
- $T$ – długość horyzontu czasowego (liczba kroków w przód)
- $\ell(x_t, u_t)$ – koszt etapowy w chwili $t$ (np. odchylenie od toru, zużycie energii)
- $\Phi(x_T)$ – koszt końcowy (np. odległość od celu na końcu horyzontu)
- $x_t$ – stan systemu w czasie $t$
- $u_t$ – sterowanie w czasie $t$

Ten wzór wyraża ogólną postać celu w optymalnym sterowaniu – sumę kosztów pośrednich i końcowego. Zarówno klasyczne MPC, jak i MPPI, starają się zminimalizować tę wartość.

### 3.2 Ważenie trajektorii (metoda całki po ścieżkach)

$$w_i = \frac{\exp\left(-\frac{1}{\lambda}(J_i - \min(J))\right)}{\sum_j \exp\left(-\frac{1}{\lambda}(J_j - \min(J))\right)}$$


**Opis symboli:**
- $w_i$ – waga przypisana $i$-tej trajektorii
- $J_i$ – koszt $i$-tej trajektorii
- $\lambda$ – parametr eksploracji (tzw. temperatura)
- $\min(J)$ – najniższy koszt spośród wszystkich trajektorii

Ten wzór implementuje tzw. **soft-minimum**, czyli wersję uśrednioną minimum z uwzględnieniem wszystkich trajektorii, gdzie lepsze (o niższym koszcie) są silniej premiowane. Jest to kluczowy element MPPI – odróżnia go od MPC, który wybiera jedną, najbardziej optymalną trajektorię.

### 3.3 Aktualizacja sterowania

$$ u_t^{\text{new}} = u_t^{\text{nom}} + \sum_i w_i \cdot \varepsilon_i(t) $$


**Opis symboli:**
- $u_t^{\text{new}}$ – nowe sterowanie w chwili $t$
- $u_t^{\text{nom}}$ – nominalne sterowanie (punkt odniesienia)
- $\varepsilon_i(t)$ – szum dodany do $i$-tej trajektorii w chwili $t$
- $w_i$ – waga $i$-tej trajektorii



Wzór ten mówi, że nowe sterowanie jest korektą nominalnej trajektorii na podstawie losowych prób i ich jakości. Dzięki temu MPPI nie potrzebuje analitycznych pochodnych i może działać z nieliniowymi, nieciągłymi modelami.

### Porównanie z klasycznym MPC

W klasycznym Model Predictive Control, rozwiązujemy deterministyczny problem optymalizacji:

$$\min_{u_0, \dots, u_T} J$$


przy zadanych ograniczeniach i modelu. Takie podejście wymaga:
- obliczania gradientów,
- dokładnego modelu matematycznego,
- spełnienia warunków wypukłości lub różniczkowalności.

MPPI omija te wymagania, korzystając z **probabilistycznej estymacji optimum** na podstawie wielu symulacji. Jest więc bardziej elastyczne, ale wymaga większych zasobów obliczeniowych.

## 4. Zastosowanie MPPI

### 4.1 Symulacje i robotyka

MPPI wykorzystywane jest m.in. do:

- sterowania pojazdami autonomicznymi (off-road, tor wyścigowy),
- sterowania robotami mobilnymi (Turtlebot3),
- integracji z ROS 2 Navigation Stack (plugin MPPI),
- planowania ruchu w środowiskach takich jak Gazebo czy CARLA.


## 5. Podsumowanie

MPPI to elastyczna, skalowalna metoda sterowania optymalnego, umożliwiająca działanie w czasie rzeczywistym i zastosowanie w złożonych, nieliniowych układach. Dzięki swojej stochastycznej naturze, MPPI potrafi omijać lokalne minima i dobrze radzi sobie z trudnymi warunkami środowiskowymi, jak np. jazda z poślizgiem.

---
