import torch
import numpy as np

# Referencyjna trajektoria (nadpisywana z main.py)
centerline = None
centerline_tensor = None
prev_idx = 0


def set_centerline(ref_points):
    global centerline, centerline_tensor
    centerline = ref_points
    centerline_tensor = torch.tensor(ref_points, dtype=torch.float32)


def get_nearest_point(x, y):
    global prev_idx
    SEARCH_IDX_LEN = 200

    if prev_idx >= centerline_tensor.shape[0]:
        prev_idx = centerline_tensor.shape[0] - 1

    start_idx = prev_idx
    end_idx = min(prev_idx + SEARCH_IDX_LEN, centerline_tensor.shape[0])

    dx = centerline_tensor[start_idx:end_idx, 0] - x
    dy = centerline_tensor[start_idx:end_idx, 1] - y
    dists = dx**2 + dy**2
    min_local_idx = torch.argmin(dists)
    nearest_idx = start_idx + min_local_idx

    nearest_idx = min(nearest_idx, centerline_tensor.shape[0] - 1)
    prev_idx = nearest_idx

    return centerline_tensor[nearest_idx]


def clamp_control(u):
    # Smooth control with simple moving average (along trajectory horizon)
    kernel_size = 5
    if u.shape[0] >= kernel_size:
        weights = torch.ones(kernel_size, device=u.device) / kernel_size
        smoothed = torch.nn.functional.conv1d(
            u.transpose(0, 1).unsqueeze(0),  # [1, 2, T]
            weights.view(1, 1, -1).repeat(u.shape[1], 1, 1),
            padding=kernel_size // 2,
            groups=u.shape[1]
        ).squeeze(0).transpose(0, 1)
        u = smoothed

    u_clamped = torch.zeros_like(u)
    u_clamped[:, 0] = torch.clamp(u[:, 0], -0.5, 0.5)
    u_clamped[:, 1] = torch.clamp(u[:, 1], -0.2, 0.5)
    return u_clamped


def vehicle_dynamics(state, action):
    action = clamp_control(action)

    lF, lR = 0.9, 0.9
    m, Iz = 200.0, 100.0
    g = 9.81
    Cm, Cr0, Cr2 = 400.0, 50.0, 1.0
    ptv = 10.0
    BF, CF, DF = 5.0, 1.3, 1.0
    BR, CR, DR = 5.0, 1.3, 1.0

    s, n, mu, vx, vy, r, delta, T = state.unbind(dim=-1)
    delta_dot, T_dot = action.unbind(dim=-1)

    T = torch.clamp(T, 0.0, 0.5)
    vx = torch.clamp(vx, 0.5, 6.0)
    delta = torch.clamp(delta, -0.5, 0.5)
    mu = torch.remainder(mu + np.pi, 2 * np.pi) - np.pi

    alpha_f = torch.atan2(vy + lF * r, vx) - delta
    alpha_r = torch.atan2(vy - lR * r, vx)

    Fn_f = lR / (lF + lR) * m * g
    Fn_r = lF / (lF + lR) * m * g

    Fy_f = Fn_f * DF * torch.sin(CF * torch.atan(BF * alpha_f))
    Fy_r = Fn_r * DR * torch.sin(CR * torch.atan(BR * alpha_r))

    Fx = Cm * T - Cr0 - Cr2 * vx**2
    rt = torch.tan(delta) * vx / (lF + lR)
    Mtv = ptv * (rt - r)

    ds = (vx * torch.cos(mu) - vy * torch.sin(mu))
    dn = (vx * torch.sin(mu) + vy * torch.cos(mu))
    dmu = r
    dvx = (Fx - Fy_f * torch.sin(delta) + m * vy * r) / m
    dvy = (Fy_r + Fy_f * torch.cos(delta) - m * vx * r) / m
    dr = (Fy_f * lF * torch.cos(delta) - Fy_r * lR + Mtv) / Iz

    return torch.stack([ds, dn, dmu, dvx, dvy, dr, delta_dot, T_dot], dim=-1)


def running_cost(state, action):
    x = state[:, 0]
    y = state[:, 1]
    yaw = state[:, 2]
    v = state[:, 3]

    cost = torch.zeros(state.shape[0], dtype=torch.float32, device=state.device)
    #print("[DEBUG] running_cost: state.shape =", state.shape)
    for i in range(state.shape[0]):
        x_i = x[i].item()
        y_i = y[i].item()
        yaw_i = (yaw[i].item() + 2 * np.pi) % (2 * np.pi)
        v_i = v[i].item()

        ref = get_nearest_point(x_i, y_i)
        ref_x = ref[0].item()
        ref_y = ref[1].item()
        ref_yaw = (ref[2].item() + 2 * np.pi) % (2 * np.pi)
        ref_v = ref[3].item()

        dx = x_i - ref_x
        dy = y_i - ref_y
        dyaw = yaw_i - ref_yaw
        dv = v_i - ref_v

        state_cost = 50.0 * dx**2 + 50.0 * dy**2 + 1.0 * dyaw**2 + 20.0 * dv**2
        u_cost = torch.sum(action[i] ** 2) * 5.0  # waga sterowania
        cost[i] = state_cost + u_cost
        #print(f"[DEBUG] running_cost[{i}]: state_cost={state_cost:.4f}, u_cost={u_cost:.4f}, total={cost[i]:.4f}")

    return cost


def terminal_cost(state, action=None):
    if state.ndim == 4 and state.shape[0] == 1:
        state = state[0]
    if state.ndim == 3:
        state = state[:, -1]

    x = state[:, 0]
    y = state[:, 1]
    yaw = state[:, 2]
    v = state[:, 3]

    cost = torch.zeros(state.shape[0], dtype=torch.float32, device=state.device)
    #print("[DEBUG] terminal_cost: state.shape =", state.shape)
    for i in range(state.shape[0]):
        x_i = x[i].item()
        y_i = y[i].item()
        yaw_i = (yaw[i].item() + 2 * np.pi) % (2 * np.pi)
        v_i = v[i].item()

        ref = get_nearest_point(x_i, y_i)
        ref_x = ref[0].item()
        ref_y = ref[1].item()
        ref_yaw = (ref[2].item() + 2 * np.pi) % (2 * np.pi)
        ref_v = ref[3].item()

        dx = x_i - ref_x
        dy = y_i - ref_y
        dyaw = yaw_i - ref_yaw
        dv = v_i - ref_v

        terminal = 50.0 * dx**2 + 50.0 * dy**2 + 1.0 * dyaw**2 + 20.0 * dv**2
        cost[i] = terminal
        #print(f"[DEBUG] terminal_cost[{i}]: terminal={terminal:.4f}")

    return cost
