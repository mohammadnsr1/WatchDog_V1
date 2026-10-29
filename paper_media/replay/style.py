"""Colors, fonts and sizes shared by the video, the figure and the web export."""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Optional

# Lane order, top to bottom, and the display names used in the data flow figure.
LANES = ("executor", "bus", "monitors", "watchdog")
LANE_TITLES = {
    "executor": "Executor Agent",
    "bus": "ROS 2 Event Bus (/watchdog/events)",
    "monitors": "Deterministic Monitors",
    "watchdog": "WatchDog Agent",
}
LANE_SHORT = {"executor": "Executor", "bus": "Bus", "monitors": "Monitor", "watchdog": "WatchDog"}

# Palette (hex). Executor blue, WatchDog and monitors green, fire and FAIL red,
# timeout gray, normal end of step dark green, event bus neutral gray.
EXECUTOR_BLUE = "#2f6fb5"
WATCHDOG_GREEN = "#2e9e5b"
MONITOR_GREEN = "#47b36f"
FAIL_RED = "#d63a3a"
TIMEOUT_GRAY = "#8c8c8c"
END_STEP_DARK_GREEN = "#1d6b3c"
BUS_GRAY = "#6e7781"
INK = "#1f2328"
MUTED = "#57606a"
PANEL_BG = "#f6f8fa"
CANVAS_BG = "#ffffff"
GRID = "#d0d7de"
HIGHLIGHT_BG = "#fff4cc"
PASS_GREEN = WATCHDOG_GREEN

LANE_COLOR = {
    "executor": EXECUTOR_BLUE,
    "bus": BUS_GRAY,
    "monitors": MONITOR_GREEN,
    "watchdog": WATCHDOG_GREEN,
}


def event_color(lane: str, kind: str, label: str = "") -> str:
    """Marker color for one event."""
    if kind == "fire" or kind == "halt":
        return FAIL_RED
    if kind in ("verdict", "run_verdict"):
        return FAIL_RED if label.startswith("FAIL") else (
            TIMEOUT_GRAY if label.startswith("INCONCLUSIVE") else PASS_GREEN)
    if kind == "timeout":
        return TIMEOUT_GRAY
    if kind == "end_step":
        return END_STEP_DARK_GREEN
    return LANE_COLOR.get(lane, BUS_GRAY)


def hex_to_rgb(h: str) -> tuple[int, int, int]:
    h = h.lstrip("#")
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


# ---------------------------------------------------------------------------
# Fonts
# ---------------------------------------------------------------------------

# One sans serif family, first one found wins. DejaVu Sans ships with matplotlib,
# so the last candidate always resolves.
FONT_CANDIDATES_REGULAR = (
    "Inter-Regular.ttf", "LiberationSans-Regular.ttf", "DejaVuSans.ttf",
)
FONT_CANDIDATES_BOLD = (
    "Inter-Bold.ttf", "LiberationSans-Bold.ttf", "DejaVuSans-Bold.ttf",
)
FONT_SEARCH_DIRS = (
    Path("/usr/share/fonts"), Path("/usr/local/share/fonts"), Path.home() / ".fonts",
    Path.home() / ".local" / "share" / "fonts",
)
MPL_FONT_FAMILY = ["Inter", "Liberation Sans", "DejaVu Sans"]


@lru_cache(maxsize=4)
def _find_font_file(bold: bool) -> Optional[str]:
    names = FONT_CANDIDATES_BOLD if bold else FONT_CANDIDATES_REGULAR
    for name in names:
        for d in FONT_SEARCH_DIRS:
            if d.exists():
                hit = next(iter(d.rglob(name)), None)
                if hit is not None:
                    return str(hit)
    try:  # matplotlib bundles DejaVu Sans
        import matplotlib
        p = Path(matplotlib.get_data_path()) / "fonts" / "ttf" / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")
        if p.exists():
            return str(p)
    except ImportError:
        pass
    return None


@lru_cache(maxsize=64)
def pil_font(size: int, bold: bool = False):
    from PIL import ImageFont
    path = _find_font_file(bold)
    if path:
        return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def apply_matplotlib_style() -> None:
    import matplotlib as mpl
    mpl.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": MPL_FONT_FAMILY,
        "font.size": 7,
        "axes.titlesize": 7.5,
        "axes.labelsize": 7,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "pdf.fonttype": 42,   # TrueType embedded in the PDF
        "ps.fonttype": 42,
        "svg.fonttype": "path",  # glyphs as paths: renders identically everywhere
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.6,
        "lines.linewidth": 0.9,
    })


# ---------------------------------------------------------------------------
# Video geometry (1920 x 1080)
# ---------------------------------------------------------------------------

VIDEO_W, VIDEO_H, FPS = 1920, 1080, 30
MARGIN = 24
FRAME_BOX = (MARGIN, MARGIN, MARGIN + 1128, MARGIN + 570)          # ~60% x 55%
EXEC_PANEL = (FRAME_BOX[2] + 24, MARGIN, VIDEO_W - MARGIN, MARGIN + 273)
WD_PANEL = (FRAME_BOX[2] + 24, MARGIN + 297, VIDEO_W - MARGIN, FRAME_BOX[3])
SWIM_BOX = (MARGIN, FRAME_BOX[3] + 24, VIDEO_W - MARGIN, FRAME_BOX[3] + 24 + 320)
SIGNAL_BOX = (MARGIN, SWIM_BOX[3] + 12, VIDEO_W - MARGIN, VIDEO_H - MARGIN)
LANE_LABEL_W = 250
SWIM_WINDOW_S = 30.0       # real seconds visible around the playhead
PANEL_LINES = 5            # most recent events per agent panel
CARD_VIDEO_S = 4.0         # how long a verdict / fire card stays, video seconds
EXCERPT_VIDEO_S = 5.0      # how long a prompt excerpt card stays, video seconds
LOOK_POPUP_VIDEO_S = 3.0
FLASH_VIDEO_S = 1.2
CAPTION_VIDEO_S = 5.0

# Figure widths, inches (IEEE two column page).
FIG_WIDTH_IN = {"double": 7.16, "single": 3.5}
