#!/usr/bin/env python3
"""
Draw a bird's-eye preview of the generated parkur, straight from the same
centerline code the world is built with.

    python3 tools/preview_track.py            # writes tools/track_preview.png

This exists because opening Gazebo is a slow way to answer "is the layout
right?". The preview reads config.yaml, walks the centerline, and draws the
road edges, the barrier lines and the numbered stages - so a layout mistake
shows up in a second rather than after a rebuild.

Needs matplotlib:  pip install matplotlib
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "generator"))

import yaml                                                # noqa: E402
from centerline import Profile, build_from_layout, resolve_pads   # noqa: E402
from generate import accel_geometry, apply_profile                # noqa: E402

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is required:  pip install matplotlib")

HERE = pathlib.Path(__file__).resolve().parent
CFG = HERE.parent / "generator" / "config.yaml"
OUT = HERE / "track_preview.png"


def main() -> int:
    cfg = yaml.safe_load(CFG.read_text(encoding="utf-8"))
    layout, _ = resolve_pads(cfg["layout"], cfg["track"]["row_length"])
    cl = build_from_layout(layout, cfg["track"]["turn_radius"],
                           profile=Profile())
    apply_profile(cfg, cl)

    half = cfg["track"]["road_width"] / 2.0
    edge = half + cfg["track"]["barrier_thickness"] / 2.0

    n = 2000
    ss = [cl.total_length * i / n for i in range(n + 1)]
    frames = [cl.frame_at(s) for s in ss]

    cx = [f.x for f in frames]
    cy = [f.y for f in frames]
    lx, ly, rx, ry = [], [], [], []
    for f in frames:
        px, py, _ = f.offset(lateral=+edge)
        lx.append(px); ly.append(py)
        px, py, _ = f.offset(lateral=-edge)
        rx.append(px); ry.append(py)

    fig, (ax, axz) = plt.subplots(
        2, 1, figsize=(15, 11),
        gridspec_kw=dict(height_ratios=[3.1, 1.0]))
    ax.fill(lx + rx[::-1], ly + ry[::-1], color="#d9d9d9", zorder=1,
            label=f"road, {cfg['track']['road_width']} m")
    ax.plot(lx, ly, color="#c02020", lw=1.6, zorder=3, label="barriers")
    ax.plot(rx, ry, color="#c02020", lw=1.6, zorder=3)
    ax.plot(cx, cy, color="#808080", lw=0.7, ls=":", zorder=2)

    # Numbered stage markers, plus start and finish.
    for p in cl.primitives:
        f = cl.frame_at(p.s0 + p.length / 2.0)
        if p.stage is not None:
            ox, oy, _ = f.offset(lateral=edge + 2.2)
            ax.plot([f.x], [f.y], marker="o", ms=4, color="#c02020", zorder=5)
            ax.annotate(f"{p.stage}", (ox, oy), ha="center", va="center",
                        fontsize=11, fontweight="bold", color="white", zorder=6,
                        bbox=dict(boxstyle="circle,pad=0.32", fc="#c02020",
                                  ec="white", lw=1.4))
            ax.annotate(p.label, (ox, oy), xytext=(0, -16),
                        textcoords="offset points", ha="center", fontsize=7.5,
                        color="#333333", zorder=6)

    f0, f1 = cl.frame_at(0.0), cl.frame_at(cl.total_length)
    for f, txt, col in ((f0, "BASLANGIC", "#1a7f37"), (f1, "BITIS", "#0b3d91")):
        ax.plot([f.x], [f.y], marker="*", ms=16, color=col, zorder=7)
        ax.annotate(txt, (f.x, f.y), xytext=(0, 14), textcoords="offset points",
                    ha="center", fontsize=8.5, fontweight="bold", color=col)

    # Travel-direction arrows.
    for s in [cl.total_length * k / 22 for k in range(1, 22)]:
        f = cl.frame_at(s)
        ax.arrow(f.x, f.y, 2.4 * math.cos(f.yaw), 2.4 * math.sin(f.yaw),
                 head_width=0.9, head_length=1.1, fc="#4a4a4a", ec="#4a4a4a",
                 lw=0.5, zorder=4, length_includes_head=True)

    # S6.11 hizlanma parkuru, which lives outside the centerline entirely.
    g = accel_geometry(cfg, cl)
    ax.add_patch(plt.Rectangle((g["x_start"], g["y_south"]),
                               g["x_finish"] - g["x_start"], g["width"],
                               fc="#d9d9d9", ec="#c02020", lw=1.6, zorder=2))
    ax.add_patch(plt.Rectangle((g["x_finish"], g["y_south"]),
                               g["x_end"] - g["x_finish"], g["width"],
                               fc="#ececec", ec="#c02020", lw=1.6, zorder=2,
                               hatch="///"))
    for k in range(1, cfg["accel_strip"]["lanes"]):
        yy = g["y_south"] + k * cfg["accel_strip"]["lane_width"]
        ax.plot([g["x_start"], g["x_end"]], [yy, yy], color="#ffffff", lw=1.0,
                zorder=3)
    for xx, lbl in ((g["x_start"], "hizlanma\nbaslangic"),
                    (g["x_finish"], "30 m\nbitis")):
        ax.plot([xx, xx], [g["y_south"], g["y_north"]], color="#0b3d91",
                lw=2.0, zorder=4)
        ax.annotate(lbl, (xx, g["y_north"]), xytext=(0, 8),
                    textcoords="offset points", ha="center", fontsize=7.5,
                    fontweight="bold", color="#0b3d91")
    ax.annotate(f"hizlanma parkuru  -  {cfg['accel_strip']['lanes']} serit,"
                f" {cfg['accel_strip']['length']:.0f} m + "
                f"{cfg['accel_strip']['stop_zone']:.0f} m durma",
                ((g["x_start"] + g["x_end"]) / 2, g["y_centre"]),
                ha="center", va="center", fontsize=8, color="#444444")
    lx += [g["x_start"], g["x_end"]]; rx += [g["x_start"], g["x_end"]]
    ly += [g["y_south"] - 4, g["y_north"] + 6]
    ry += [g["y_south"] - 4, g["y_north"] + 6]

    # Stage labels sit 2.2 m outside the barrier, so the axes need headroom or
    # the bottom row's labels get clipped off the figure.
    pad = 7.0
    ax.set_xlim(min(lx + rx) - pad, max(lx + rx) + pad)
    ax.set_ylim(min(ly + ry) - pad, max(ly + ry) + pad)

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25, ls="--", lw=0.5)
    ax.set_xlabel("x  [m]  (east)")
    ax.set_ylabel("y  [m]  (north)")
    ax.set_title(f"TEKNOFEST 2026 IKA parkuru - centerline "
                 f"{cl.total_length:.1f} m, {len(cl.stages())} stages",
                 fontsize=12, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)

    # ---- elevation and bank along the track -----------------------------
    zs = [f.z for f in frames]
    rolls = [math.degrees(f.roll) for f in frames]

    axz.fill_between(ss, 0, zs, color="#b9b9b9", zorder=1)
    axz.plot(ss, zs, color="#333333", lw=1.5, zorder=3, label="height  z [m]")

    axr = axz.twinx()
    axr.plot(ss, rolls, color="#c07000", lw=1.5, ls="--", zorder=3,
             label="bank [deg]")
    axr.set_ylabel("bank  [deg]", color="#c07000", fontsize=9)
    axr.tick_params(axis="y", labelcolor="#c07000", labelsize=8)

    for p in cl.primitives:
        if p.stage is None:
            continue
        mid = p.s0 + p.length / 2.0
        axz.axvspan(p.s0, p.s0 + p.length, color="#c02020", alpha=0.06, zorder=0)
        axz.annotate(f"{p.stage}", (mid, 0), xytext=(0, -14),
                     textcoords="offset points", ha="center", fontsize=8,
                     fontweight="bold", color="#c02020")

    ramp = cfg["ramp"]["grade_percent"]
    bank = cfg["side_slope"]["grade_percent"]
    axz.set_xlim(0, cl.total_length)
    axz.set_xlabel("s  [m]  along the centerline")
    axz.set_ylabel("height  [m]", fontsize=9)
    axz.grid(True, alpha=0.25, ls="--", lw=0.5)
    axz.set_title(f"elevation and bank   -   ramps {ramp:.0f} %  "
                  f"({math.degrees(math.atan(ramp / 100)):.2f} deg),   "
                  f"yan egim {bank:.0f} %  "
                  f"({math.degrees(math.atan(bank / 100)):.2f} deg)",
                  fontsize=10)
    h1, l1 = axz.get_legend_handles_labels()
    h2, l2 = axr.get_legend_handles_labels()
    axz.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8)

    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    print(f"wrote {OUT}")

    print(f"\n  centerline : {cl.total_length:.2f} m")
    print(f"  {'segment':<10} {'stage':>5} {'s0':>8} {'s1':>8}   "
          f"{'mid x':>8} {'mid y':>8}  heading")
    for p in cl.primitives:
        f = cl.frame_at(p.s0 + p.length / 2.0)
        st = str(p.stage) if p.stage is not None else "-"
        print(f"  {p.id:<10} {st:>5} {p.s0:>8.2f} {p.s0 + p.length:>8.2f}   "
              f"{f.x:>8.2f} {f.y:>8.2f}  {math.degrees(f.yaw) % 360:>6.1f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
