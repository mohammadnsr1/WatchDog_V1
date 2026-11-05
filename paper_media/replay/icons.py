"""Small line icons shared by the hand-over figure (matplotlib) and the video card (Pillow).

Each icon is a list of primitives in a unit square, y up: ("line", points),
("poly", points) filled, ("circle", (cx, cy, r)), ("rect", (x0, y0, x1, y1)) outline.
Drawn in one color at a given pixel/inch size, so they look the same in both outputs.
"""
from __future__ import annotations

ICONS: dict[str, list] = {
    # clipboard: board, clip, three lines
    "clipboard": [("rect", (0.15, 0.05, 0.85, 0.90)), ("rect", (0.36, 0.84, 0.64, 0.98)),
                  ("line", [(0.30, 0.66), (0.70, 0.66)]), ("line", [(0.30, 0.50), (0.70, 0.50)]),
                  ("line", [(0.30, 0.34), (0.58, 0.34)])],
    # ruler with ticks
    "ruler": [("rect", (0.05, 0.30, 0.95, 0.70)), ("line", [(0.25, 0.30), (0.25, 0.48)]),
              ("line", [(0.45, 0.30), (0.45, 0.55)]), ("line", [(0.65, 0.30), (0.65, 0.48)]),
              ("line", [(0.85, 0.30), (0.85, 0.55)])],
    # robot arm: base, two links, gripper
    "arm": [("rect", (0.10, 0.02, 0.50, 0.14)), ("line", [(0.30, 0.14), (0.30, 0.52)]),
            ("circle", (0.30, 0.52, 0.06)), ("line", [(0.30, 0.52), (0.72, 0.78)]),
            ("circle", (0.72, 0.78, 0.06)), ("line", [(0.72, 0.78), (0.92, 0.62)]),
            ("line", [(0.86, 0.72), (0.98, 0.72)]), ("line", [(0.86, 0.52), (0.98, 0.52)])],
    # eye
    "eye": [("line", [(0.02, 0.50), (0.25, 0.78), (0.50, 0.86), (0.75, 0.78), (0.98, 0.50),
                      (0.75, 0.22), (0.50, 0.14), (0.25, 0.22), (0.02, 0.50)]),
            ("circle", (0.50, 0.50, 0.17)), ("poly", [(0.44, 0.56), (0.56, 0.56), (0.56, 0.44), (0.44, 0.44)])],
    # play triangle
    "play": [("poly", [(0.22, 0.10), (0.22, 0.90), (0.88, 0.50)])],
    # lightning bolt
    "bolt": [("poly", [(0.58, 0.98), (0.20, 0.46), (0.48, 0.46), (0.40, 0.02), (0.80, 0.56), (0.52, 0.56)])],
    # shield with check
    "shield": [("line", [(0.50, 0.98), (0.10, 0.82), (0.12, 0.40), (0.50, 0.04), (0.88, 0.40), (0.90, 0.82), (0.50, 0.98)]),
               ("line", [(0.32, 0.50), (0.45, 0.36), (0.70, 0.66)])],
    # megaphone
    "megaphone": [("poly", [(0.08, 0.40), (0.08, 0.62), (0.36, 0.62), (0.80, 0.90), (0.80, 0.12), (0.36, 0.40)]),
                  ("line", [(0.86, 0.36), (0.96, 0.36)]), ("line", [(0.86, 0.66), (0.96, 0.66)]),
                  ("line", [(0.18, 0.40), (0.24, 0.12)])],
    # gate: two posts and a raised bar
    "gate": [("line", [(0.14, 0.05), (0.14, 0.95)]), ("line", [(0.86, 0.05), (0.86, 0.95)]),
             ("line", [(0.14, 0.62), (0.86, 0.62)]), ("line", [(0.14, 0.40), (0.86, 0.40)])],
    # gavel: head and handle
    "gavel": [("rect", (0.42, 0.56, 0.90, 0.84)), ("line", [(0.56, 0.56), (0.12, 0.12)]),
              ("line", [(0.04, 0.06), (0.30, 0.06)])],
}


def draw_mpl(ax, name: str, x: float, y: float, size: float, color: str, lw: float = 0.9) -> None:
    """Draw at lower-left (x, y) in axis units, `size` wide and high."""
    from matplotlib.patches import Circle, Polygon, Rectangle
    for kind, g in ICONS[name]:
        if kind == "line":
            ax.plot([x + px * size for px, _ in g], [y + py * size for _, py in g], color=color, lw=lw,
                    solid_capstyle="round", solid_joinstyle="round")
        elif kind == "poly":
            ax.add_patch(Polygon([(x + px * size, y + py * size) for px, py in g], closed=True,
                                 facecolor=color, edgecolor=color, lw=lw * 0.5))
        elif kind == "circle":
            cx, cy, r = g
            ax.add_patch(Circle((x + cx * size, y + cy * size), r * size, facecolor="none", edgecolor=color, lw=lw))
        elif kind == "rect":
            x0, y0, x1, y1 = g
            ax.add_patch(Rectangle((x + x0 * size, y + y0 * size), (x1 - x0) * size, (y1 - y0) * size,
                                   facecolor="none", edgecolor=color, lw=lw))


def draw_pil(d, name: str, x: int, y: int, size: int, color, width: int = 3) -> None:
    """Draw at top-left (x, y) in pixels (Pillow y grows down)."""
    def P(px, py):
        return (x + px * size, y + (1 - py) * size)
    for kind, g in ICONS[name]:
        if kind == "line":
            pts = [P(*p) for p in g]
            d.line(pts, fill=color, width=width, joint="curve")
            for p in (pts[0], pts[-1]):
                d.ellipse((p[0] - width / 2, p[1] - width / 2, p[0] + width / 2, p[1] + width / 2), fill=color)
        elif kind == "poly":
            d.polygon([P(*p) for p in g], fill=color)
        elif kind == "circle":
            cx, cy, r = g
            c = P(cx, cy)
            d.ellipse((c[0] - r * size, c[1] - r * size, c[0] + r * size, c[1] + r * size), outline=color, width=width)
        elif kind == "rect":
            x0, y0, x1, y1 = g
            a, b = P(x0, y1), P(x1, y0)
            d.rectangle((a[0], a[1], b[0], b[1]), outline=color, width=width)
