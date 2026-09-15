#!/usr/bin/env python3
"""Draw a clear vector 3-D mission schematic for one IEEE column."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Ellipse, FancyArrowPatch, Rectangle


HERE = Path(__file__).resolve().parent
BLUE = "#0969da"
ORANGE = "#d95f02"
TEAL = "#00a6a6"
GREEN = "#159947"
NAVY = "#17365d"
GRAY = "#aab2ba"


def world_track(t, z=5.0):
    """A horizontal figure eight in world coordinates."""
    return 8.0 * np.sin(t), 4.0 * np.sin(2.0 * t), np.full_like(t, z)


def project(x, y, z):
    """Oblique projection chosen for a legible one-column figure."""
    return np.asarray(x) + 0.43 * np.asarray(y), np.asarray(z) + 0.25 * np.asarray(y)


def point(t, z=5.0):
    x, y, zz = world_track(np.asarray([t]), z)
    u, v = project(x, y, zz)
    return float(u[0]), float(v[0])


def drone(ax, xy, payload=False, color=NAVY, scale=0.34, zorder=8):
    """Small vector quadrotor glyph that remains sharp in the PDF."""
    u, v = xy
    ax.plot([u - scale, u + scale], [v - 0.18 * scale, v + 0.18 * scale],
            color=color, lw=1.15, zorder=zorder)
    ax.plot([u - scale, u + scale], [v + 0.18 * scale, v - 0.18 * scale],
            color=color, lw=1.15, zorder=zorder)
    for du, dv in ((-scale, -0.18 * scale), (scale, 0.18 * scale),
                   (-scale, 0.18 * scale), (scale, -0.18 * scale)):
        ax.add_patch(Ellipse((u + du, v + dv), 0.28, 0.075,
                             facecolor="#8fa4b5", edgecolor=color,
                             lw=0.65, zorder=zorder + 1))
    ax.add_patch(Rectangle((u - 0.12, v - 0.085), 0.24, 0.17,
                           facecolor=color, edgecolor="white", lw=0.4,
                           zorder=zorder + 2))
    if payload:
        ax.plot([u, u], [v - 0.10, v - 0.29], color="#555", lw=0.7,
                zorder=zorder)
        ax.add_patch(Rectangle((u - 0.13, v - 0.52), 0.26, 0.22,
                               facecolor="#c58b3c", edgecolor="#704d1f",
                               lw=0.65, zorder=zorder + 1))


def arrow_on_track(ax, t0, color=BLUE):
    a = point(t0)
    b = point(t0 + 0.15)
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=8,
                                 lw=1.0, color=color, zorder=7))


def main():
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans", "Arial"],
        "font.size": 7.0,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })

    fig, ax = plt.subplots(figsize=(3.55, 2.25))

    # Sparse projected floor grid gives the height cues without visual clutter.
    for gx in np.linspace(-10, 10, 6):
        u, v = project([gx, gx], [-5.5, 5.5], [0, 0])
        ax.plot(u, v, color="#e7ebef", lw=0.45, zorder=0)
    for gy in np.linspace(-5, 5, 5):
        u, v = project([-10, 10], [gy, gy], [0, 0])
        ax.plot(u, v, color="#e7ebef", lw=0.45, zorder=0)

    t = np.linspace(-np.pi / 2, 3 * np.pi / 2, 700)
    x, y, z = world_track(t)
    u, v = project(x, y, z)
    ug, vg = project(x, y, np.zeros_like(z))
    ax.plot(ug, vg, color=GRAY, lw=0.8, ls=(0, (4, 3)), alpha=0.7, zorder=1)
    ax.plot(u, v, color=BLUE, lw=2.2, solid_capstyle="round", zorder=5)

    # Pickup and vertical climb join the nominal loop at its leftmost point.
    pickup = project(-8.0, 0.0, 0.20)
    join = point(-np.pi / 2)
    ax.plot([pickup[0], join[0]], [pickup[1], join[1]], color=BLUE,
            lw=2.2, solid_capstyle="round", zorder=5)
    ax.add_patch(Circle(pickup, 0.38, facecolor=GREEN, edgecolor=GREEN,
                        lw=0.8, alpha=0.20, zorder=2))

    carry_t = -0.45
    release_t = 2.18
    carry = point(carry_t)
    release = point(release_t)

    # Release is a highlighted portion of the uninterrupted nominal loop.
    ts = np.linspace(release_t - 0.12, release_t + 0.15, 55)
    xs, ys, zs = world_track(ts)
    us, vs = project(xs, ys, zs)
    ax.plot(us, vs, color=ORANGE, lw=2.6, solid_capstyle="round", zorder=6)

    # Optional recovery branches away from the loop; it is dashed throughout.
    hover = (10.0, 3.15)
    q = np.linspace(0.0, 1.0, 100)
    control = np.array([7.6, 3.00])
    start = np.array(release)
    end = np.array(hover)
    branch = ((1 - q) ** 2)[None, :] * start[:, None]
    branch += (2 * (1 - q) * q)[None, :] * control[:, None]
    branch += (q ** 2)[None, :] * end[:, None]
    ax.plot(branch[0], branch[1], color=TEAL, lw=1.65,
            ls=(0, (4, 3)), zorder=4)

    # Height guides and four mission snapshots.
    for xy in (carry, release, hover):
        ax.plot([xy[0], xy[0]], [project(xy[0], 0, 0)[1], xy[1]],
                color=GRAY, lw=0.65, ls=(0, (3, 3)), alpha=0.65, zorder=2)
    drone(ax, pickup, payload=True)
    drone(ax, carry, payload=True)
    drone(ax, release, payload=False)
    drone(ax, hover, payload=False, color=TEAL)

    # Released parcel.
    ax.plot([release[0], release[0]], [release[1] - 0.12, release[1] - 0.65],
            color=ORANGE, lw=0.85, ls=(0, (3, 2)), zorder=4)
    ax.add_patch(Rectangle((release[0] - 0.13, release[1] - 0.91), 0.26, 0.22,
                           facecolor="#c58b3c", edgecolor="#704d1f",
                           lw=0.65, zorder=6))

    # Direction arrows show that the blue trajectory continues after release.
    for ta in (-1.25, -0.62, 0.20, 1.18, 2.48, 3.55, 4.42):
        arrow_on_track(ax, ta)
    j = 62
    ax.add_patch(FancyArrowPatch(branch[:, j], branch[:, j + 7],
                                 arrowstyle="-|>", mutation_scale=7,
                                 lw=0.9, color=TEAL, zorder=6))

    # Few, large labels are more legible than the raster labels in the old image.
    ax.annotate("1  Pickup", pickup, xytext=(-7.55, 0.58), textcoords="data",
                color=NAVY, weight="bold", fontsize=6.8,
                arrowprops=dict(arrowstyle="-", color=NAVY, lw=0.65))
    ax.annotate("2  Carry", carry, xytext=(-6.25, 3.18), textcoords="data",
                color=NAVY, weight="bold", fontsize=6.8,
                arrowprops=dict(arrowstyle="-", color=NAVY, lw=0.65))
    ax.annotate("3  Release", release, xytext=(3.55, 2.86), textcoords="data",
                color=NAVY, weight="bold", fontsize=6.8,
                arrowprops=dict(arrowstyle="-", color=NAVY, lw=0.65))
    ax.annotate("Optional hover\nif unresolved", hover,
                xytext=(7.15, 1.22), textcoords="data", ha="left",
                color=TEAL, weight="bold", fontsize=6.1,
                arrowprops=dict(arrowstyle="-", color=TEAL, lw=0.65))

    # Compact oblique coordinate triad.
    origin = (-10.0, -0.75)
    ax.annotate("", xy=(-8.7, -0.75), xytext=origin,
                arrowprops=dict(arrowstyle="-|>", color="#c52b29", lw=1.0))
    ax.annotate("", xy=(-9.45, -0.18), xytext=origin,
                arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.0))
    ax.annotate("", xy=(-10.0, 0.15), xytext=origin,
                arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.0))
    ax.text(-8.58, -0.81, "$x$", fontsize=6.6)
    ax.text(-9.42, -0.10, "$y$", fontsize=6.6)
    ax.text(-10.12, 0.24, "$z$", fontsize=6.6)

    ax.set_xlim(-10.7, 11.0)
    ax.set_ylim(-1.15, 6.45)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    fig.subplots_adjust(left=0.005, right=0.995, bottom=0.015, top=0.995)
    fig.savefig(HERE / "fig_track_3d_vector.pdf", bbox_inches="tight", pad_inches=0.025)
    fig.savefig(HERE / "fig_track_3d_vector.png", dpi=450,
                bbox_inches="tight", pad_inches=0.025)


if __name__ == "__main__":
    main()
