"""Pure logic of mp_fake_base.py (no rclpy)."""
import math


def integrate(x, y, yaw, vx, vy, wz, dt):
    """Exact-enough body-twist integration over dt (midpoint heading); returns the new (x, y, yaw)."""
    mid = yaw + 0.5 * wz * dt
    return (x + (vx * math.cos(mid) - vy * math.sin(mid)) * dt,
            y + (vx * math.sin(mid) + vy * math.cos(mid)) * dt,
            yaw + wz * dt)
