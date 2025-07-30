import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np


def draw_court_with_shooting_zone(
    court_length: float = 15,
    court_width: float = 8,
    min_shot_distance: float = 2.1,
    max_shot_distance: float = 5.5,
    hoop_x: float = 0.975,
    hoop_y: float = 4,
    save_path: str | None = None,
):
    """Draw a basketball half-court and highlight the valid shooting zone.

    Parameters
    ----------
    court_length : float, default 15
        Length of the court (in metres).
    court_width : float, default 8
        Width of the court (in metres).
    min_shot_distance : float, default 2
        Minimum distance from the hoop from which shots are allowed (in metres).
    max_shot_distance : float, default 5.5
        Maximum distance from the hoop from which shots are allowed (in metres).
    hoop_x, hoop_y : float | None, default None
        Absolute position of the hoop (metres). If either is ``None``, the hoop
        defaults to坐
        the centre point ``(court_length / 2, court_width / 2)``.
    save_path : str | None
        If provided, the plot will also be saved to this location.
    """

    # Calculate hoop (basket) position
    if hoop_x is None:
        hoop_x = court_length / 2
    if hoop_y is None:
        hoop_y = court_width / 2

    fig, ax = plt.subplots(figsize=(court_length * 0.6, court_width * 0.6))

    # --- Court outline ---
    court = Rectangle((0, 0), court_length, court_width, linewidth=2, edgecolor="black", facecolor="none")
    ax.add_patch(court)

    # --- Shooting zone ---
    # Draw outer circle (max distance)
    outer_circle = Circle((hoop_x, hoop_y), max_shot_distance, color="orange", alpha=0.3, zorder=2)
    # Draw inner circle (min distance) – same color as background so we get an annulus
    inner_circle = Circle((hoop_x, hoop_y), min_shot_distance, color="white", zorder=3)

    # Clip the shooting zone so it does not extend outside the court rectangle
    outer_circle.set_clip_path(court)
    inner_circle.set_clip_path(court)

    ax.add_patch(outer_circle)
    ax.add_patch(inner_circle)

    # ========== Three-point like semi-circle (radius 3.1 m) ==========
    arc_radius = 3.1
    # Draw only the half facing the court interior (x >= hoop_x)
    theta = np.linspace(-np.pi / 2, np.pi / 2, 180)
    arc_x = hoop_x + arc_radius * np.cos(theta)
    arc_y = hoop_y + arc_radius * np.sin(theta)
    arc_path = np.vstack((arc_x, arc_y)).T
    ax.plot(arc_x, arc_y, color="blue", linewidth=2, zorder=5)

    # Straight lines connecting arc to positions 0.9 m from sidelines
    y_lower = 0.9
    y_upper = court_width - 0.9
    # Arc endpoints (should coincide theoretically)
    end_lower = (hoop_x + np.sqrt(max(0, arc_radius**2 - (y_lower - hoop_y) ** 2)), y_lower)
    end_upper = (hoop_x + np.sqrt(max(0, arc_radius**2 - (y_upper - hoop_y) ** 2)), y_upper)

    # Draw straight segments from hoop_x to the arc endpoints (vertical lines)
    ax.plot([hoop_x, end_lower[0]], [y_lower, y_lower], color="blue", linewidth=2, zorder=5)
    ax.plot([hoop_x, end_upper[0]], [y_upper, y_upper], color="blue", linewidth=2, zorder=5)

    # --- Mid-court line ---
    ax.plot(
        [court_length / 2, court_length / 2],  # x-coordinates
        [0, court_width],                      # y-coordinates
        color="black",
        linewidth=2,
        zorder=4,
    )

    # --- Hoop marker (red dot, diameter 0.45 m) ---
    hoop_marker = Circle((hoop_x, hoop_y), 0.225, facecolor="red", edgecolor="red", zorder=6)
    ax.add_patch(hoop_marker)

    # --- Formatting ---
    ax.set_aspect("equal")
    ax.set_xlim(-1, court_length + 1)
    ax.set_ylim(-1, court_width + 1)
    ax.axis("off")
    ax.set_title("Basketball Court with Valid Shooting Zone", pad=20)

    if save_path:
        plt.savefig(save_path, bbox_inches="tight", dpi=300)
    plt.show()


if __name__ == "__main__":
    draw_court_with_shooting_zone()
