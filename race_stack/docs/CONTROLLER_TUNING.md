# 🏎️ Controller Tuning & Optimization Guide

This guide explains the parameters available in `rqt_reconfigure` for the `/controller` node. Adjusting these values allows you to trade off between **lap time (speed)** and **stability (tracking accuracy)**.

---

## 📐 Lateral Control (L1 Guidance)
The car uses an L1-style controller (similar to Pure Pursuit). It calculates a "Lookahead Distance" ($L_1$) and steers the car toward a point on the track at that distance.

### 1. `m_l1` (Speed Gain)
*   **What it does:** Scales the lookahead distance based on your current speed ($L_1 = q_{l1} + speed \times m_{l1}$).
*   **When to increase:** If the car is "fishtailing" or oscillating on straights at high speed. A higher value makes the car smoother and more stable.
*   **When to decrease:** If the car is "lazy" and misses the apex of turns. A lower value makes the steering more aggressive and reactive.
*   **Optimal Tuning:** Start at `0.3`. Increase in `0.05` increments if you see high-frequency oscillations in your steering plot.

### 2. `t_clip_min` (The "Floor")
*   **What it does:** Sets the absolute minimum lookahead distance, regardless of how slow the car is going.
*   **Why it matters:** If this is too high (e.g., `1.0`), the car will look too far ahead in tight hairpins and "cut" the corner or hit the inside wall.
*   **Optimal Tuning:** Set this to roughly your car's wheelbase or slightly more (e.g., `0.4` to `0.6`). If you have high lateral error in slow corners, **lower this first**.

### 3. `t_clip_max` (The "Ceiling")
*   **What it does:** Prevents the lookahead from becoming infinitely large at high speeds.
*   **Why it matters:** If the car looks *too* far ahead at 10m/s, it will start turning for the next corner while it's still in the middle of a straight.
*   **Optimal Tuning:** Usually set to `5.0` or `8.0`.

---

## ⚡ Longitudinal Control (Speed)
These parameters decide how much the car should "brake" or slow down based on its performance.

### 4. `lat_err_coeff` (The "Brake" for Error)
*   **What it does:** Reduces the commanded speed if the car is far away from the raceline.
*   **Why adjust:** It’s a safety mechanism. If the car is 20cm off-path, it should slow down to regain control before it hits a wall.
*   **When to increase:** If your car is flying off the track in corners because it's going too fast for its current steering capability.
*   **Optimal Tuning:** `1.0` is standard. Increase to `2.0` for high-speed testing on new tracks.

### 5. `speed_lookahead`
*   **What it does:** Tells the car to look $X$ seconds into the future when deciding what speed to drive.
*   **Why it matters:** Real cars have physical inertia. You need to "look ahead" at an upcoming corner and start slowing down *before* you reach it.
*   **Optimal Tuning:** `0.0` (instant) to `0.2` seconds.

---

## 👥 Trailing Controller (Overtaking & Drafting)
Used when the `state_machine` detects an opponent and switches to `TRAILING` mode.

### 6. `trailing_gap`
*   **What it does:** The desired safety distance (in meters) to keep behind the opponent.
*   **Optimal Tuning:** `1.5` meters is a good balance for drafting without crashing.

### 7. `trailing_p_gain`
*   **What it does:** How hard the car accelerates/brakes to maintain that gap.
*   **Symptoms of bad tuning:** If the car "jerks" or pumps the throttle while following an opponent, this gain is too high.

---

## 🛠️ Summary Diagnostic Table

| Symptom | Primary Suspect | Fix |
| :--- | :--- | :--- |
| **Oscillations on straights** | `m_l1` | Increase slightly |
| **Car cuts corners (inside wall)**| `t_clip_min` | Decrease |
| **Car goes wide (outside wall)** | `m_l1` | Decrease |
| **Car overshoots brake zones** | `speed_lookahead`| Increase |
| **Car unstable at low speed** | `q_l1` | Increase |

---
*Note: Always use the `plot_controller_data.py` tool after a run to see if your changes actually reduced the **Lateral Error**.*
