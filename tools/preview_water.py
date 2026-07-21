#!/usr/bin/env python3
"""
Cross-sections through the su gecisi basin, read back out of the generated SDF.

    python3 tools/preview_water.py     # writes tools/water_preview.png

Reads the world file rather than the config on purpose. The config says what
was intended; the SDF is what Gazebo will actually load. Anything that went
wrong between the two shows up here as a gap, an overlap, or a floating box.
"""

from __future__ import annotations

import math
import pathlib
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "generator"))

import yaml                                                       # noqa: E402
from centerline import Profile, build_from_layout, resolve_pads   # noqa: E402
from generate import apply_profile, _water_span                   # noqa: E402

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except ImportError:
    sys.exit("matplotlib is required:  pip install matplotlib")

HERE = pathlib.Path(__file__).resolve().parent
CFG = HERE.parent / "generator" / "config.yaml"
SDF = HERE.parent / "worlds" / "ika_parkur.sdf"
OUT = HERE / "water_preview.png"

COLOURS = {
    "ground": ("#9a9a86", "earth"),
    "parkur_road": ("#59595e", "road"),
    "parkur_barriers": ("#b0b0b4", "barrier"),
    "su_havuzu_duvar": ("#3a3a3e", "basin wall"),
    "su_havuzu": ("#2e6b9e", "water"),
}


def boxes(world):
    """Every axis-aligned-ish box in the world as (model, centre, size, yaw)."""
    for m in world.findall("model"):
        mp = [float(v) for v in (m.find("pose").text.split()
                                 if m.find("pose") is not None else "0 0 0 0 0 0".split())]
        for link in m.findall("link"):
            lp = [float(v) for v in (link.find("pose").text.split()
                                     if link.find("pose") is not None
                                     else "0 0 0 0 0 0".split())]
            for vis in link.findall("visual"):
                box = vis.find("geometry").find("box")
                if box is None:
                    continue
                size = [float(v) for v in box.find("size").text.split()]
                c = [mp[i] + lp[i] for i in range(3)]
                rot = [mp[i + 3] + lp[i + 3] for i in range(3)]
                yield m.get("name"), c, size, rot


def main() -> int:
    cfg = yaml.safe_load(CFG.read_text(encoding="utf-8"))
    layout, _ = resolve_pads(cfg["layout"], cfg["track"]["row_length"])
    cl = build_from_layout(layout, cfg["track"]["turn_radius"], profile=Profile())
    apply_profile(cfg, cl)
    d0, d1 = _water_span(cfg, cl)

    world = ET.parse(SDF).getroot().find("world")
    all_boxes = list(boxes(world))

    fa, fb = cl.frame_at(d0), cl.frame_at(d1)
    x_lo, x_hi = sorted((fa.x, fb.x))
    x_lo, x_hi = x_lo - 3.0, x_hi + 3.0
    cut_x = cl.frame_at((d0 + d1) / 2.0).x          # transverse cut mid-pool

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9))

    # ---------- longitudinal, sliced on the centerline y ------------------
    for name, c, s, rot in all_boxes:
        if abs(c[1]) > s[1] / 2.0:                   # not crossing y = 0
            continue
        col, _lbl = COLOURS.get(name, ("#cccccc", name))
        pitch = rot[1]
        if abs(pitch) < 1e-6:
            ax1.add_patch(Rectangle((c[0] - s[0] / 2, c[2] - s[2] / 2), s[0], s[2],
                                    fc=col, ec="none", alpha=0.95, zorder=2))
        else:                                        # sloped slab: draw rotated
            import numpy as np
            hw, hh = s[0] / 2, s[2] / 2
            pts = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
            r = np.array([[math.cos(-pitch), -math.sin(-pitch)],
                          [math.sin(-pitch), math.cos(-pitch)]])
            pts = pts @ r.T + np.array([c[0], c[2]])
            ax1.fill(pts[:, 0], pts[:, 1], fc=col, ec="none", alpha=0.95, zorder=2)

    ss = [d0 - 3 + (d1 - d0 + 6) * i / 600 for i in range(601)]
    ax1.plot([cl.frame_at(s).x for s in ss], [cl.frame_at(s).z for s in ss],
             color="#c02020", lw=1.4, ls="--", zorder=6, label="road surface")
    ax1.axhline(0, color="#1f6fb0", lw=1.2, ls=":", zorder=5, label="water line z=0")
    ax1.set_xlim(x_hi, x_lo)
    ax1.set_ylim(-1.3, 1.1)
    ax1.set_aspect("equal")
    ax1.grid(alpha=0.25, ls="--", lw=0.5)
    ax1.set_xlabel("x  [m]   (direction of travel  <--)")
    ax1.set_ylabel("z  [m]")
    ax1.set_title(f"S6.3 su gecisi - longitudinal section on the centerline   "
                  f"({cfg['water']['depth'] * 100:.0f} cm deep, "
                  f"{cfg['water']['length']:.0f} m pool, "
                  f"{cfg['water']['bank_slope_length']:.1f} m banks)",
                  fontsize=11, fontweight="bold")
    ax1.legend(loc="upper right", fontsize=8)

    # ---------- transverse, sliced mid-pool -------------------------------
    for name, c, s, rot in all_boxes:
        if abs(c[0] - cut_x) > s[0] / 2.0:
            continue
        col, _lbl = COLOURS.get(name, ("#cccccc", name))
        ax2.add_patch(Rectangle((c[1] - s[1] / 2, c[2] - s[2] / 2), s[1], s[2],
                                fc=col, ec="#00000030", lw=0.4, alpha=0.95, zorder=2))
    ax2.axhline(0, color="#1f6fb0", lw=1.2, ls=":", zorder=5)
    ax2.set_xlim(-3.2, 3.2)
    ax2.set_ylim(-1.3, 1.1)
    ax2.set_aspect("equal")
    ax2.grid(alpha=0.25, ls="--", lw=0.5)
    ax2.set_xlabel("y  [m]   (across the road)")
    ax2.set_ylabel("z  [m]")
    ax2.set_title("transverse section through the middle of the pool",
                  fontsize=11, fontweight="bold")

    handles = [plt.Line2D([], [], marker="s", ls="", ms=10, mfc=c, mec="none",
                          label=l) for c, l in COLOURS.values()]
    ax2.legend(handles=handles, loc="upper right", fontsize=8, ncol=2)

    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
