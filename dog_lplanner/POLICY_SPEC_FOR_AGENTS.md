# SAC Path-Planning Policy — Spec for LLM Agents

Minimal specification for this policy. Use it when writing inference, export, lidar integration, and related code.

--- Quick reference ---
• Actor input: [56] = base [47] + action_history [9]. Base: lidar(40) + w(1) + sin(1) + cos(1) + dist(1) + prev(3). No vx, vy.
• Actor output: [3] in [-1, 1] → scale by cmd_scale [0.8, 0.4, 0.35].
• Lidar: 40 sectors, raw [0.25, 3.0] m → normalize to [-1, 1] (-1=close, +1=far). Collision: min(raw) < 0.35.
• Config: configs/a1.yaml. history_length=3, critic_history_length=3.
--- end ---

---

## 1. Training

- **Algorithm:** SAC (Soft Actor-Critic).
- **Environment:** MuJoCo, Unitree A1, cluttered scene (boxes). Robot and goal spawn at free locations.
- **Two control levels:**
  1. **SAC (high-level):** from observations outputs velocity commands `[vx, vy, w]` in `[-1, 1]`.
  2. **Walking policy (low-level):** pretrained MLP. Input: `velocity_commands` (from SAC) + base_ang_vel, projected_gravity, joint_pos, joint_vel, last_action. Output: 12 target joint angles. A PD controller tracks them.
- **Rates:** sim 500 Hz, control 50 Hz, SAC 10 Hz (every `control_decimation * sac_decimation` steps). Lidar fed to SAC is cached at the SAC rate.
- **Episode:** runs up to `max_steps`; ends on `reached` (distance to goal < 0.2 m), `collision` (see lidar), or timeout.

Config: `configs/a1.yaml`. Key fields: `cmd_scale`, `sac.history_length`, `sac.critic_history_length`, `reward_weights`, `lidar.noise_std`.

---

## 2. Observations

### 2.1 Base Actor observation (47 features)

Order is fixed:

| Indices | Size | Description |
|--------|------|-------------|
| 0–39   | 40   | Lidar per sector (normalized; see §4) |
| 40     | 1    | Angular rate `w` (normalized by `max_angular_vel`, typically `cmd_scale[2]`) |
| 41     | 1    | `sin(angle_to_target)` in robot frame |
| 42     | 1    | `cos(angle_to_target)` in robot frame |
| 43     | 1    | Goal distance (normalized by `max_distance`, then `*2 - 1` → `[-1, 1]`) |
| 44–46  | 3    | Previous SAC command `[vx, vy, w]` in `[-1, 1]` |

- **Important:** the Actor **does not** get `vx`, `vy` (linear velocities). Only `w`, lidar, goal terms, and `prev_action`.
- `angle_to_target`, `distance`: from `get_target_info(robot_pos, target_pos, robot_quat)`. In the robot frame: x forward, y left. `sin = dy_local/distance`, `cos = dx_local/distance`.

### 2.2 Action history (Actor)

- `sac.history_length = 3`.
- Network input: `base(47) + history(3 * 3) = 56` scalars.
- History: last 3 commands `[vx, vy, w]`, chronological (oldest first):  
  `[vx_{t-3}, vy_{t-3}, w_{t-3}, vx_{t-2}, vy_{t-2}, w_{t-2}, vx_{t-1}, vy_{t-1}, w_{t-1}]`.
- If history is short, **left-pad** with zeros.

### 2.3 Critic

- Base: Actor(47) + `vx`(1) + `vy`(1) = 49. Plus `critical_topk` nearest lidar rays and observation history (`critic_history_length`). Inference only needs the Actor.

### 2.4 Normalization

- Lidar: §4.
- `w`: `clip(w, -max_angular_vel, max_angular_vel) / max_angular_vel` → `[-1, 1]`.
- `distance`: `clip(d, 0, max_distance) / max_distance` then `*2 - 1` → `[-1, 1]`. `max_distance` ≈ 10.75 m (room diagonal).
- `sin` / `cos`: already in `[-1, 1]`; `clip(..., -1, 1)` if needed.
- `prev_action`: stored and fed in `[-1, 1]`.

---

## 3. Actions

- **SAC output:** `[vx_cmd, vy_cmd, w_cmd]` in `[-1, 1]`.
- **Scaling (required):**  
  `vx = vx_cmd * cmd_scale[0]`, `vy = vy_cmd * cmd_scale[1]`, `w = w_cmd * cmd_scale[2]`.  
  In `a1.yaml`: `cmd_scale: [0.8, 0.4, 0.35]` (m/s, m/s, rad/s).
- These `[vx, vy, w]` go to the walking policy as `velocity_commands`. The walking policy outputs 12 joint targets; do not change it—only provide correct inputs.

---

## 4. Lidar in training and inference

### 4.1 Raw data

- 40 rays (MuJoCo lidar sensors) or equivalent (e.g. 2D projection of a point cloud).
- Units: meters. Range `[min_range, max_range]` = `[0.25, 3.0]` m. Closer than 0.25 m is treated as robot body and dropped.
- `inf` / `nan` / negatives: replaced early in the pipeline (e.g. `fix_negative_lidar_values`: neighbors or 3.0), then `clip` to `[0, 3]`.

### 4.2 Sectors (`process_lidar_to_sectors`)

- 40 sectors, angles uniform on `[-π, π]`. Sector `i`:  
  `[-π + i * (2π/40), -π + (i+1) * (2π/40))`, last sector includes `π`.
- Per sector: **minimum** range among rays falling in that sector. If none: `max_range` (3.0 m).
- Output: length-40 vector (meters).

### 4.3 Noise (optional)

- In training (and optionally in sim): Gaussian `N(0, lidar.noise_std)` on raw ranges, then `clip` to `[0, max_range]`. In `a1.yaml`: `noise_std: 0.02`.

### 4.4 Normalization for the policy

- Per sector:  
  `x = clip(dist, 0, max_range) / max_range` → `[0, 1]`,  
  then `x * 2 - 1` → `[-1, 1]`.  
  **-1 = close, +1 = far / no obstacle.**

### 4.5 Collision and reward

- **Collision:** `min(valid lidar ranges) < 0.35` m → end episode, large penalty (`reward_weights.collision`, e.g. -500).
- Reward uses **raw** lidar (meters), including obstacle terms. The policy observation always uses **normalized** lidar (sectors in `[-1, 1]`).

### 4.6 ROS1 (real robot)

- `dog_lplanner/scripts/lidar_processor_ros1.py`: LaserScan → 40 sectors (meters), same per-sector minimum and range as `process_lidar_to_sectors`. Normalization to `[-1, 1]` is in `inference_onnx.py`.

---

## 5. Common pitfalls

1. **Actor without vx/vy:** observation is only `w`, lidar, goal, `prev_action`. Do not add linear velocities to the Actor.
2. **Action history:** exactly the last 3 `[vx, vy, w]`, old → new, left-pad with zeros. Actor input size with `history_length=3`: 56.
3. **cmd_scale:** always apply to SAC output; in config `[0.8, 0.4, 0.35]`—do not confuse with older values like 1.7/1.5.
4. **Lidar:** network gets normalized 40 values in `[-1, 1]`; collision/reward use raw meters; sectors must follow the same scheme (40 sectors, min, 0.25–3.0 m).
5. **Goal:** `distance`, `sin`, `cos` in the robot frame; if `distance < 1e-6`, use `sin=0`, `cos=1`.
6. **Rate:** policy runs at 10 Hz; lidar and goal data should be time-consistent (including with an external lidar).

---

## 6. Files and config

- Config: `configs/a1.yaml`.
- Observations: `src/utils/observation.py` — `build_actor_observation`, `process_lidar_to_sectors`, `fix_negative_lidar_values`.
- Reward: `src/utils/reward.py` — `compute_reward_reference*`; lidar in meters.
- Goal: `src/utils/target_generator.py` — `get_target_info`.
- Training: `scripts/train.py`; ONNX export: `scripts/export_to_onnx.py`; inference: `scripts/inference_onnx.py`.

When implementing inference, export, or lidar integration, match this spec: observation layout (47 + 9 when history is 3), feature order, normalizations, and `cmd_scale`.
