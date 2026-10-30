"""Entry point.

    python -m paper_media.replay.cli summary <session_dir> [--camera main]
    python -m paper_media.replay.cli preview <story.yaml> --frames 20 --out <dir>
    python -m paper_media.replay.cli video   <story.yaml> --out <file.mp4>
    python -m paper_media.replay.cli figure  <story.yaml> --out <file.pdf> [--width double|single]
    python -m paper_media.replay.cli web     <story1.yaml> [<story2.yaml> ...] --site <SITE_REPO>
    python -m paper_media.replay.cli package <story.yaml> [...] [--out-root paper_media/outputs] [--no-video]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="paper_media.replay.cli")
    sub = ap.add_subparsers(dest="mode", required=True)
    p = sub.add_parser("summary", help="counts per lane and kind, time span, unparsed records")
    p.add_argument("session_dir", type=Path)
    p.add_argument("--camera", default="main")
    p = sub.add_parser("preview", help="render N frames as PNGs to check the layout")
    p.add_argument("story", type=Path)
    p.add_argument("--frames", type=int, default=20)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("video", help="render the MP4 replay")
    p.add_argument("story", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("figure", help="render the static PDF and SVG figure")
    p.add_argument("story", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--width", choices=("double", "single"), default="double")
    p = sub.add_parser("web", help="export videos, posters and run JSON into the website repository")
    p.add_argument("stories", type=Path, nargs="+")
    p.add_argument("--site", type=Path, required=True)
    p = sub.add_parser("package", help="figure, key frames, video and media.yaml into paper_media/outputs/<media_id>/")
    p.add_argument("stories", type=Path, nargs="+")
    p.add_argument("--out-root", type=Path, default=Path(__file__).resolve().parents[1] / "outputs")
    p.add_argument("--no-video", action="store_true")
    a = ap.parse_args(argv)

    if a.mode == "summary":
        from .load_session import session_summary
        session_summary(a.session_dir, camera=a.camera)
        return 0
    if a.mode == "web":
        from .export_web import SiteError, export_web
        try:
            export_web(a.stories, a.site)
        except SiteError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        return 0

    from .load_session import load_story
    from .timeline import prepare
    if a.mode == "package":
        from .package import package
        for sp in a.stories:
            try:
                package(load_story(sp), a.out_root, video=not a.no_video)
            except ValueError as exc:
                print(f"error: {exc}", file=sys.stderr)
                return 2
        return 0
    story = load_story(a.story)
    if a.mode == "preview":
        from .render_video import render_preview
        render_preview(prepare(story), a.frames, a.out)
    elif a.mode == "video":
        from .render_video import render_video
        render_video(prepare(story), a.out)
    elif a.mode == "figure":
        from .render_figure import render_figure
        render_figure(prepare(story), a.out, a.width)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
