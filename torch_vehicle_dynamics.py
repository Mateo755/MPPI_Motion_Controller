import torch
import numpy as np

def vehicle_dynamics(state, action):
    # Parametry pojazdu
    lF, lR = 0.9, 0.9
    m, Iz = 200.0, 100.0
    g = 9.81
    Cm, Cr0, Cr2 = 400.0, 50.0, 1.0
    ptv = 10.0
    BF, CF, DF = 5.0, 1.3, 1.0
    BR, CR, DR = 5.0, 1.3, 1.0

    # Rozpakowanie
    s, n, mu, vx, vy, r, delta, T = state.unbind(dim=-1)
    delta_dot, T_dot = action.unbind(dim=-1)

    # Ograniczenia
    T = torch.clamp(T, 0.0, 0.5)
    vx = torch.clamp(vx, 0.5, 6.0)
    delta = torch.clamp(delta, -0.5, 0.5)
    mu = torch.remainder(mu + np.pi, 2 * np.pi) - np.pi

    # Siły boczne
    alpha_f = torch.atan2(vy + lF * r, vx) - delta
    alpha_r = torch.atan2(vy - lR * r, vx)

    Fn_f = lR / (lF + lR) * m * g
    Fn_r = lF / (lF + lR) * m * g

    Fy_f = Fn_f * DF * torch.sin(CF * torch.atan(BF * alpha_f))
    Fy_r = Fn_r * DR * torch.sin(CR * torch.atan(BR * alpha_r))

    Fx = Cm * T - Cr0 - Cr2 * vx**2
    rt = torch.tan(delta) * vx / (lF + lR)
    Mtv = ptv * (rt - r)

    # Równania ruchu
    ds = (vx * torch.cos(mu) - vy * torch.sin(mu))
    dn = (vx * torch.sin(mu) + vy * torch.cos(mu))
    dmu = r
    dvx = (Fx - Fy_f * torch.sin(delta) + m * vy * r) / m
    dvy = (Fy_r + Fy_f * torch.cos(delta) - m * vx * r) / m
    dr = (Fy_f * lF * torch.cos(delta) - Fy_r * lR + Mtv) / Iz

    result = torch.stack([ds, dn, dmu, dvx, dvy, dr, delta_dot, T_dot], dim=-1)

    if not torch.isfinite(result).all():
        print("NaN w dynamice!")
        print("state:", state)
        print("action:", action)
        print("result:", result)

    return result

def running_cost(state, action):
    n = state[:, 1]
    mu = state[:, 2]
    vx = state[:, 3]
    vy = state[:, 4]
    r = state[:, 5]
    delta = state[:, 6]
    T = state[:, 7]

    # Upewnienie się, że mu jest w zakresie [-pi, pi]
    mu = (mu + np.pi) % (2 * np.pi) - np.pi

    # Prędkość wzdłuż trajektorii
    s_dot = vx * torch.cos(mu) - vy * torch.sin(mu)

    # Klampowanie nietypowych wartości vx, vy, r itd. jeśli chcesz dodatkowe zabezpieczenia
    vx = torch.clamp(vx, 0.0, 100.0)
    vy = torch.clamp(vy, -50.0, 50.0)
    r = torch.clamp(r, -10.0, 10.0)
    delta = torch.clamp(delta, -1.0, 1.0)
    T = torch.clamp(T, 0.0, 1.0)

    i = 0  # indeks pierwszego wiersza
    print(f"vx: {state[i, 3].item():.2f}, vy: {state[i, 4].item():.2f}, r: {state[i, 5].item():.2f}, mu: {state[i, 2].item():.2f}, s_dot: {(vx[i] * torch.cos(mu[i]) - vy[i] * torch.sin(mu[i])).item():.2f}")

    # Koszt z karą za niską prędkość (s_dot bliskie zeru może być niepożądane)
    cost = (
        5.0 * n**2 +                   
        10.0 * mu**2 +                 
        4.0 * vy**2 +                  
        0.5 * (r - mu)**2 +            
        0.1 * delta**2 +               
        0.1 * T**2 +                   
        2.0 * torch.relu(vx - 4.0)**2 - 
       -10.0 * torch.where(s_dot > 0.0, s_dot, torch.zeros_like(s_dot))                   
    )

    # Diagnostyka przy NaN
    if not torch.isfinite(cost).all():
        print("NaN w koszcie!")
        for i in range(min(5, state.shape[0])):  # diagnostyka pierwszych 5 elementów
            print(f"[{i}] vx: {vx[i].item():.2f}, vy: {vy[i].item():.2f}, r: {r[i].item():.2f}, mu: {mu[i].item():.2f}, s_dot: {s_dot[i].item():.2f}")
    
    cost = torch.nan_to_num(cost, nan=1e6, posinf=1e6, neginf=1e6)

    return cost



