#!/usr/bin/env python3
"""
Check the generated world against the sartname, clause by clause.

    python3 tools/verify.py          # exits non-zero if anything fails

Reads worlds/ika_parkur.sdf, NOT config.yaml. The config records what was
intended; the SDF is what Gazebo actually loads. Everything measurable is
measured out of the world file or the texture PNGs, so a generator bug that
quietly drops or misplaces something shows up here rather than at the
competition.
"""

from __future__ import annotations

import math
import pathlib
import sys
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG / "generator"))

import yaml                                                       # noqa: E402
from centerline import Profile, build_from_layout, resolve_pads   # noqa: E402
from generate import (accel_geometry, apply_profile, slider_slot,  # noqa: E402
                      stop_line_stations, water_hole, _water_span)

CFG = PKG / "generator" / "config.yaml"
SDF = PKG / "worlds" / "ika_parkur.sdf"
MESHES = PKG / "models" / "ika_isaretler" / "meshes"

GREEN, RED, DIM, OFF = "\033[32m", "\033[31m", "\033[2m", "\033[0m"
if not sys.stdout.isatty():
    GREEN = RED = DIM = OFF = ""

results = []


def check(clause, what, got, want, ok, unit=""):
    results.append(ok)
    tick = f"{GREEN}pass{OFF}" if ok else f"{RED}FAIL{OFF}"
    g = f"{got}{unit}" if not isinstance(got, float) else f"{got:g}{unit}"
    w = f"{want}{unit}" if not isinstance(want, float) else f"{want:g}{unit}"
    print(f"  {tick}  {clause:<8} {what:<34} {g:>16}   {DIM}want {w}{OFF}")


def near(a, b, tol=1e-6):
    return abs(a - b) <= tol


def north_face(link):
    """Northernmost world y touched by a box link, from its pose and size.

    Not just `y + size_y / 2`: barriers through the turns carry yaw and the
    ones on the yan egim carry roll, so the half-extents have to be rotated
    into world axes first. Summing |R_y . half| gives the link's AABB in y,
    which is what "how close does the wall come" means for a world-axis edge.
    """
    _px, py, _pz, roll, pitch, yaw = [float(v)
                                      for v in link.find("pose").text.split()]
    hx, hy, hz = [0.5 * float(v) for v in
                  link.find("collision").find("geometry").find("box")
                  .find("size").text.split()]
    sr, cr, sp, cp, sy, cy = (math.sin(roll), math.cos(roll),
                              math.sin(pitch), math.cos(pitch),
                              math.sin(yaw), math.cos(yaw))
    return py + (abs(sy * cp) * hx
                 + abs(sy * sp * sr + cy * cr) * hy
                 + abs(sy * sp * cr - cy * sr) * hz)


def main() -> int:
    cfg = yaml.safe_load(CFG.read_text(encoding="utf-8"))
    layout, _ = resolve_pads(cfg["layout"], cfg["track"]["row_length"])
    cl = build_from_layout(layout, cfg["track"]["turn_radius"], profile=Profile())
    apply_profile(cfg, cl)
    world = ET.parse(SDF).getroot().find("world")
    models = {m.get("name"): m for m in world.findall("model")}
    t = cfg["track"]

    print(f"\n{'=' * 92}\n TEKNOFEST 2026 IKA parkuru - sartname compliance"
          f"\n world: {SDF}\n{'=' * 92}\n")

    # ---- 6.1 parkur ----------------------------------------------------
    print(" 6.1  YARISMA PARKURU")
    check("6.1", "road width", t["road_width"], 3.0,
          near(t["road_width"], 3.0), " m")
    check("6.1", "barrier height", t["barrier_height"], "0.80 +/- 0.10",
          0.70 <= t["barrier_height"] <= 0.90, " m")
    check("6.1", "straight sections", t["straight_length"], 20.0,
          near(t["straight_length"], 20.0), " m")
    gap = slider_slot(cfg, cl)
    maxgap = 0.0
    prev = {}
    half = t["road_width"] / 2 + t["barrier_thickness"] / 2
    for s0, s1, _p in cl.steps(t["tile_length"], t["barrier_arc_step_deg"],
                               breakpoints=list(gap)):
        if gap[0] <= 0.5 * (s0 + s1) <= gap[1]:
            prev = {}
            continue
        for lat in (+half, -half):
            a = cl.frame_at(s0).offset(lateral=lat)
            b = cl.frame_at(s1).offset(lateral=lat)
            if lat in prev:
                maxgap = max(maxgap, math.dist(prev[lat], a))
            prev[lat] = b
    check("6.1", "barrier continuity (max gap)", round(maxgap * 1000, 3), 0.0,
          maxgap < 1e-6, " mm")
    # "kirmizi ve beyaz renklerde" is a requirement, not decoration: this is the
    # object the vehicle sees most and every contact costs 5 points, so a
    # perception stack tuned against grey walls is tuned against the wrong
    # world. Read straight off the diffuse colours, both barrier runs.
    def _hue(rgba):
        r, g, b, _a = [float(v) for v in rgba.split()]
        if r > 0.6 and g < 0.35 and b < 0.35:
            return "red"
        return "white" if min(r, g, b) > 0.8 else "other"
    # Per model, not pooled: a single set over both runs would pass if the main
    # track went all-red and the drag strip supplied the white.
    banding = {}
    for mdl in ("parkur_barriers", "hizlanma_bariyer"):
        banding[mdl] = {_hue(v.find("material").find("diffuse").text)
                        for l in models[mdl].findall("link")
                        for v in l.findall("visual")}
    check("6.1", "barriers are red and white",
          " ".join(f"{k.split('_')[0]}:{'/'.join(sorted(v))}"
                   for k, v in banding.items()),
          "both red/white",
          all(v == {"red", "white"} for v in banding.values()))
    rows = {}
    for it in layout:
        if it.get("row"):
            rows.setdefault(it["row"], []).append(it["id"])
    lens = {r: sum(cl.segment(i).length for i in ids) for r, ids in rows.items()}
    check("Sekil 1", "all rows equal length", f"{sorted(set(round(v,3) for v in lens.values()))}",
          f"[{t['row_length']}]", len(set(round(v, 3) for v in lens.values())) == 1)

    # ---- 6.2 tabelalar -------------------------------------------------
    print("\n 6.2  ASAMA TABELALARI")
    try:
        from PIL import Image
        im = Image.open(MESHES / "stage_01.png")
        row = [im.getpixel((x, im.height // 2)) for x in range(im.width)]
        red = [i for i, c in enumerate(row) if c[0] > 150 and c[1] < 80]
        d = (max(red) - min(red) + 1) / im.width * cfg["signs"]["outer_diameter"]
        check("6.2", "red ring outer diameter", round(d, 4), 0.60,
              near(d, 0.60, 0.005), " m")
    except ImportError:
        print(f"  {DIM}skip  6.2      texture measurement needs Pillow{OFF}")
    obj = (MESHES / "stage_01.obj").read_text()
    verts = [[float(x) for x in l.split()[1:]]
             for l in obj.splitlines() if l.startswith("v ")]
    rr = [math.hypot(v[0], v[2]) for v in verts if math.hypot(v[0], v[2]) > 1e-9]
    # Tolerance is 1e-5, not 1e-9: OBJ stores coordinates to six decimal places,
    # so a perfectly round mesh still shows sub-micrometre scatter on readback.
    check("6.2", "plate is a disc (mesh radius)", round(2 * max(rr), 4), 0.60,
          near(max(rr), min(rr), 1e-5) and near(2 * max(rr), 0.60, 1e-5), " m")
    n_signs = len([k for k in models if k.startswith("tabela_")
                   and not k.startswith("tabela_direk")])
    check("6.2", "one sign per stage", n_signs, len(cl.stages()),
          n_signs == len(cl.stages()))
    facing = 0
    for p in cl.stages():
        m = models[f"tabela_{p.stage:02d}"]
        q = [float(v) for v in m.find("pose").text.split()]
        d = (math.degrees(q[5] - math.pi / 2 - cl.frame_at(p.s0).yaw) + 540) % 360 - 180
        facing += abs(abs(d) - 180) < 1
    check("6.2", "signs face oncoming vehicle", facing, len(cl.stages()),
          facing == len(cl.stages()))
    # A post model's pose is its BASE, and the datum for a base is the GROUND,
    # not the road it stands beside. On the S6.10 ramp the road surface is
    # 2.25 m up, so a post pinned to the road surface stands on nothing at all
    # -- it hangs in the air with its plate at the right height, which looks
    # correct from the driving line and is obviously wrong from anywhere else.
    # Any base above z=0 is floating. Every *direk* in the world is covered,
    # not just the stage tabelalar: the stop plates and the drag strip stand on
    # the same ground, and the ramp ones are exactly where this bug bites.
    floating = [k for k, m in models.items() if "direk" in k
                and float(m.find("pose").text.split()[2]) > 1e-9]
    check("-", "sign posts reach the ground", len(floating), 0, not floating,
          " floating")

    # ---- 6.3 su gecisi -------------------------------------------------
    print("\n 6.3  SU GECISI")
    d0, d1 = _water_span(cfg, cl)
    depth = -min(cl.profile.height_at(d0 + (d1 - d0) * i / 800) for i in range(801))
    check("6.3", "water depth", round(depth, 4), 0.40, near(depth, 0.40), " m")
    hx0, hy0, hx1, hy1 = water_hole(cfg, cl)
    outside = 0
    for i in range(401):
        f = cl.frame_at(d0 + (d1 - d0) * i / 400)
        if f.z < -1e-6:
            for lat in (-1.5, 0, 1.5):
                px, py, _ = f.offset(lateral=lat)
                outside += not (hx0 <= px <= hx1 and hy0 <= py <= hy1)
    check("6.3", "ground hole covers the pit", outside, 0, outside == 0,
          " points outside")
    ncol = len(models["su_havuzu"].find("link").findall("collision"))
    check("-", "water is visual only", ncol, 0, ncol == 0, " collisions")
    bp = [p for p in world.findall("plugin") if "Buoyancy" in (p.get("name") or "")]
    check("-", "buoyancy interface", bp[0].find("graded_buoyancy")
          .find("density_change").find("above_depth").text if bp else "missing",
          "0", bool(bp))
    lo = min(cl.profile.height_at(cl.total_length * i / 4000)
             for i in range(4001) if not (d0 <= cl.total_length * i / 4000 <= d1))
    check("-", "track never dips below z=0 elsewhere", round(lo, 6), 0.0,
          lo >= -1e-9, " m")

    # ---- 6.5 - 6.7 -----------------------------------------------------
    print("\n 6.5 - 6.7  YAN EGIM / DIK ENGEL / KONILER")
    b0, b1 = cl.span("s3_bank")
    roll = cl.frame_at((b0 + b1) / 2).roll
    check("6.5", "side slope", round(math.tan(roll) * 100, 3), 20.0,
          near(math.tan(roll) * 100, 20.0, 1e-6), " %")
    lowedge = cl.frame_at((b0 + b1) / 2).offset(lateral=-t["road_width"] / 2)[2]
    check("-", "banked low edge at ground", round(lowedge, 5), 0.0,
          near(lowedge, 0.0, 1e-5), " m")
    vb = cfg["vertical_block"]["height"]
    check("6.6", "vertical block height", vb, 0.15, near(vb, 0.15), " m")
    c = cfg["cones"]
    check("6.7", "cone base", c["base"], "0.40 +/- 0.10",
          0.30 <= c["base"] <= 0.50, " m")
    check("6.7", "cone height", c["height"], "0.75 +/- 0.05",
          0.70 <= c["height"] <= 0.80, " m")
    ncones = len([k for k in models if k.startswith("trafik_konisi")])
    check("6.7", "cones placed", ncones, c["count"], ncones == c["count"])

    # ---- 6.8 kayar engel ----------------------------------------------
    print("\n 6.8  KAYAR ENGEL")
    sl = cfg["slider"]
    check("6.8", "blade width", sl["width"], 1.0, near(sl["width"], 1.0), " m")
    check("6.8", "speed", sl["speed"], 0.20, near(sl["speed"], 0.20), " m/s")
    inner = sl["travel"] / 2 - sl["width"] / 2
    check("6.8", "fully clears the road", round(inner, 3),
          f">= {t['road_width'] / 2}", inner >= t["road_width"] / 2 - 1e-9, " m")
    km = models["kayar_engel"]
    jt = [j for j in km.findall("joint") if j.get("type") == "prismatic"]
    check("-", "prismatic joint present", len(jt), 1, len(jt) == 1)
    near_bar = min((math.dist(
        (float(l.find("pose").text.split()[0]), float(l.find("pose").text.split()[1])),
        (cl.frame_at(cl.middle("s6_slider")).x, cl.frame_at(cl.middle("s6_slider")).y))
        for l in models["parkur_barriers"].findall("link")), default=99)
    check("-", "barrier slot is open", round(near_bar, 2), "> 1.5",
          near_bar > 1.5, " m")

    # ---- 6.9 engebeli arazi -------------------------------------------
    print("\n 6.9  ENGEBELI ARAZI")
    bm = cfg["bumps"]
    check("Sekil 4", "bump height", bm["height"], 0.05, near(bm["height"], 0.05), " m")
    check("Sekil 4", "bump base width", bm["base_width"], 0.20,
          near(bm["base_width"], 0.20), " m")
    nb = len(models["engebeli_arazi"].findall("link"))
    check("6.9", "bumps placed", nb, "> 0", nb > 0)
    lats = set()
    for l in models["engebeli_arazi"].findall("link"):
        lats.add(round(float(l.find("pose").text.split()[1]), 2))
    check("6.9", "separate left/right series", "yes" if len(lats) > 1 else "no",
          "yes", len(lats) > 1)

    # ---- 6.10 dik egimler ve atis --------------------------------------
    print("\n 6.10  DIK EGIMLER VE ATIS")
    for tag, seg in (("climb", "s8_up"), ("descent", "s10_down")):
        a, b = cl.span(seg)
        gr = abs(cl.profile.height_at(b) - cl.profile.height_at(a)) / (b - a) * 100
        check("6.10", f"{tag} grade", round(gr, 3), 45.0, near(gr, 45.0, 1e-6), " %")
    p0, p1 = cl.span("s9_plat")
    zs = [cl.profile.height_at(p0 + (p1 - p0) * i / 50) for i in range(51)]
    check("-", "plateau flat", round(max(zs) - min(zs), 6), 0.0,
          near(max(zs), min(zs), 1e-9), " m")
    nmark = len(models["stop_markings"].findall("link"))
    check("6.10", "stop marks on both slopes", nmark, 2, nmark == 2)
    # The plate has to stand BESIDE its own line and on the left of travel --
    # the stage tabelalar own the right-hand verge, so a stop plate appearing
    # there would read as a stage label. Measured in each line's own frame, so
    # it stays true whichever way round the row runs.
    beside = 0
    for tag, station in stop_line_stations(cfg, cl):
        m = models.get(f"dur_tabela_{tag}")
        if m is None:
            continue
        f = cl.frame_at(station)
        q = [float(v) for v in m.find("pose").text.split()]
        lat = (q[0] - f.x) * -math.sin(f.yaw) + (q[1] - f.y) * math.cos(f.yaw)
        along = (q[0] - f.x) * math.cos(f.yaw) + (q[1] - f.y) * math.sin(f.yaw)
        beside += lat > 0 and abs(lat) < 3.0 and abs(along) < 0.5
    check("6.10", "stop plate left of each line", beside, 2, beside == 2)
    hq = [float(v) for v in models["hedef"].find("pose").text.split()]
    plat = cl.frame_at(p1)
    rng = math.dist((hq[0], hq[1]), (plat.x, plat.y))
    check("6.10", "atis range", round(rng, 3), ">= 10", rng >= 10.0 - 1e-9, " m")
    try:
        from PIL import Image
        ti = Image.open(MESHES / "hedef.png")
        sc = ti.width / cfg["target"]["paper_w"]
        rowp = [ti.getpixel((x, ti.height // 2)) for x in range(ti.width)]
        blk = [i for i, cpx in enumerate(rowp) if sum(cpx) < 200]
        outer = (blk[-1] - blk[0] + 1) / sc
        check("Sekil 5", "target outer ring", round(outer, 4), 0.18,
              near(outer, 0.18, 0.005), " m")
    except ImportError:
        pass

    # ---- 6.11 hizlanma parkuru ----------------------------------------
    print("\n 6.11  HIZLANMA PARKURU")
    a = cfg["accel_strip"]
    g = accel_geometry(cfg, cl)
    check("6.11", "timed length", round(g["x_finish"] - g["x_start"], 3), 30.0,
          near(g["x_finish"] - g["x_start"], 30.0, 1e-6), " m")
    check("6.11", "stopping distance", round(g["x_end"] - g["x_finish"], 3), 10.0,
          near(g["x_end"] - g["x_finish"], 10.0, 1e-6), " m")
    # The verdict is computed once and used for BOTH the printed value and the
    # pass/fail flag. This check used to pass a literal True as `ok`, so it
    # could print "no" and still count as a pass -- a check that cannot fail is
    # worse than no check, because it reads like coverage.
    track_north = max(cl.frame_at(cl.total_length * i / 200).y
                      for i in range(201))
    is_outside = g["y_south"] > track_north
    check("6.11", "outside the main parkur", "yes" if is_outside else "no",
          "yes", is_outside)
    # "Ana parkur disinda" is satisfied by any positive gap, but a strip that
    # clears the main track by centimetres is not usable: the two barrier lines
    # need room between them for marshals and for recovering a stopped vehicle.
    # Measured barrier face to barrier face out of the SDF, not centreline to
    # centreline, so barrier thickness and the turn bulge both count.
    clearance = g["y_south"] - max(
        north_face(l) for l in models["parkur_barriers"].findall("link"))
    check("6.11", "clearance to main parkur", round(clearance, 3), "> 2",
          clearance > 2.0, " m")
    check("-", "lanes", a["lanes"], "> 1", a["lanes"] > 1)

    # Both ends carry the SAME number; the finish plate is that number struck
    # through. Checked by mesh, so swapping the two plates or reintroducing a
    # separate 12th sign fails here rather than at the competition.
    def _mesh(model):
        return (models[model].find("link").find("visual").find("geometry")
                .find("mesh").find("uri").text.rsplit("/", 1)[-1])
    want_a, want_b = (f"stage_{a['sign_start']:02d}.obj",
                      f"stage_{a['sign_start']:02d}_bitis.obj")
    got = (_mesh("hizlanma_tabela_baslangic"), _mesh("hizlanma_tabela_bitis"))
    check("6.11", "same number, cancelled at finish",
          f"{got[0]} / {got[1]}", f"{want_a} / {want_b}",
          got == (want_a, want_b))
    try:
        from PIL import Image
        # The slash has to be ON the plate, not just in the filename. Sampled
        # along the anti-diagonal it occupies: red the whole way on the finish
        # plate, and essentially absent on the start plate.
        def _red_on_diagonal(png):
            im = Image.open(MESHES / png).convert("RGB")
            c = im.width / 2.0
            arm = c * (1 - 2 * cfg["signs"]["ring_width"]
                       / cfg["signs"]["outer_diameter"]) * 0.99 * math.sqrt(0.5)
            n = 0
            for i in range(101):
                u = -arm + 2 * arm * i / 100
                px = im.getpixel((int(c + u), int(c - u)))
                n += px[0] > 150 and px[1] < 80
            return n
        on = _red_on_diagonal(f"stage_{a['sign_start']:02d}_bitis.png")
        off = _red_on_diagonal(f"stage_{a['sign_start']:02d}.png")
        check("Sekil 1", "finish plate is struck through", f"{on} vs {off}",
              "101 vs 0", on == 101 and off == 0, " px red on the diagonal")
    except ImportError:
        pass

    # ---- 7.4 vehicle envelope -----------------------------------------
    print("\n 7.4  ARAC ZARFI")
    inner_r = t["turn_radius"] - t["road_width"] / 2
    check("7.4", "turn inner radius", round(inner_r, 3), ">= 5.0",
          inner_r >= 5.0 - 1e-9, " m")

    # ---- world integrity ----------------------------------------------
    print("\n WORLD INTEGRITY")
    missing = []
    for m in world.findall("model"):
        for l in m.findall("link"):
            for v in l.findall("visual"):
                mesh = v.find("geometry").find("mesh")
                if mesh is not None:
                    rel = mesh.find("uri").text.replace("model://", "")
                    if not (PKG / "models" / rel).is_file():
                        missing.append(rel)
    check("-", "referenced meshes exist", len(missing), 0, not missing, " missing")
    dup = []
    for m in world.findall("model"):
        for l in m.findall("link"):
            nm = [e.get("name") for e in list(l)
                  if e.tag in ("collision", "visual")]
            if len(nm) != len(set(nm)):
                dup.append(l.get("name"))
    check("-", "no duplicate element names", len(dup), 0, not dup)
    names = [m.get("name") for m in world.findall("model")]
    check("-", "no duplicate model names", len(names) - len(set(names)), 0,
          len(names) == len(set(names)))
    nlinks = sum(len(m.findall("link")) for m in world.findall("model"))
    print(f"\n  {DIM}{len(names)} models, {nlinks} links, "
          f"{SDF.stat().st_size / 1024:.0f} KB{OFF}")

    npass, ntot = sum(results), len(results)
    bar = GREEN if npass == ntot else RED
    print(f"\n{'=' * 92}")
    print(f" {bar}{npass}/{ntot} checks passed{OFF}")
    print(f"{'=' * 92}\n")
    return 0 if npass == ntot else 1


if __name__ == "__main__":
    raise SystemExit(main())
