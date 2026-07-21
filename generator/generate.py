#!/usr/bin/env python3
"""
Generate the TEKNOFEST 2026 IKA parkur world for Gazebo Sim 8 (Harmonic).

    python3 generate.py                 # writes ../worlds/ika_parkur.sdf
    python3 generate.py --seed 7        # different cone / bump arrangement
    python3 generate.py -o /tmp/x.sdf   # write somewhere else

Every dimension comes from generator/config.yaml, which is itself annotated
with the sartname section each value was taken from.

BUILD STATUS
    Phase 1  [done]     world skeleton, solver, lighting, ground
    Phase 2  [pending]  centerline, road surface, barriers
    Phase 3  [pending]  dik engel, yan egim, tumsekler, ramps, cones
    Phase 4  [pending]  40 cm water basin + buoyancy
    Phase 5  [pending]  kayar engel
    Phase 6  [pending]  numbered stage signs, atis target
    Phase 7  [pending]  hizlanma parkuru
"""

from __future__ import annotations

import argparse
import math
import pathlib
import random
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required:  pip install pyyaml   (or: apt install python3-yaml)")

import signage
from centerline import Profile, build_from_layout, resolve_pads
from sdfkit import (Comment, E, banner, friction, material, pose,
                    sdf_document, static_box, vec)

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "config.yaml"
DEFAULT_OUT = HERE.parent / "worlds" / "ika_parkur.sdf"

# Shared palette. The user asked for untextured grey geometry; only the stage
# signs and the atis target get real textures, because the vision model has to
# read them. Everything else is a shade of grey, with hue used only to make the
# scene legible to a human watching the run.
GREY_ROAD = (0.35, 0.35, 0.37, 1.0)
GREY_LIGHT = (0.62, 0.62, 0.64, 1.0)
GREY_DARK = (0.22, 0.22, 0.24, 1.0)
GREY_GROUND = (0.45, 0.46, 0.44, 1.0)

# The barriers, the cones and the stop lines are the exceptions to "everything
# is grey". All three exist to be SEEN, and for the first two the sartname
# names the colours outright, so grey would be a deviation and not a
# simplification. No textures -- flat colour is enough for a banded barrier.
#
# S6.1 "Yol kenarindaki bariyerler KIRMIZI VE BEYAZ renklerde". The barrier is
# the object the vehicle sees more than any other and every contact costs 5
# points, so its colour is the last thing that should be a stand-in: perception
# tuned against grey walls is tuned against a world that will not be there.
BARRIER_RED = (0.81, 0.13, 0.15, 1.0)
BARRIER_WHITE = (0.93, 0.93, 0.92, 1.0)
GREY_RAMP = (0.42, 0.41, 0.39, 1.0)   # raised sections, so slopes read clearly
CONE_ORANGE = (0.90, 0.35, 0.05, 1.0)
CONE_WHITE = (0.92, 0.92, 0.92, 1.0)
MARK_WHITE = (0.95, 0.95, 0.95, 1.0)
WATER_BLUE = (0.18, 0.42, 0.62, 0.55)
SLIDER_YELLOW = (0.85, 0.72, 0.10, 1.0)   # the only moving thing on the track


# ---------------------------------------------------------------------------
#  Phase 1 - world skeleton
# ---------------------------------------------------------------------------

def build_physics(cfg) -> list:
    p = cfg["physics"]
    return [
        banner("SOLVER AND SYSTEM PLUGINS"),
        E(
            "physics",
            E("max_step_size", p["max_step_size"]),
            E("real_time_factor", p["real_time_factor"]),
            name="default_physics",
            type="ignored",
        ),
        E("gravity", vec(0, 0, p["gravity"])),
        E("plugin",
          filename="gz-sim-physics-system",
          name="gz::sim::systems::Physics"),
        E("plugin",
          filename="gz-sim-user-commands-system",
          name="gz::sim::systems::UserCommands"),
        E("plugin",
          filename="gz-sim-scene-broadcaster-system",
          name="gz::sim::systems::SceneBroadcaster"),
        Comment("Contact system: lets us score barrier and cone touches later."),
        E("plugin",
          filename="gz-sim-contact-system",
          name="gz::sim::systems::Contact"),
        Comment("Sensors system: required for any camera / lidar on your vehicle."),
        E("plugin",
          E("render_engine", cfg["render"]["engine"]),
          filename="gz-sim-sensors-system",
          name="gz::sim::systems::Sensors"),
        E("plugin",
          filename="gz-sim-imu-system",
          name="gz::sim::systems::Imu"),
    ]


def build_scene_and_light() -> list:
    return [
        banner("SCENE AND LIGHTING"),
        E(
            "scene",
            E("ambient", vec(0.55, 0.55, 0.55, 1.0)),
            E("background", vec(0.70, 0.78, 0.88, 1.0)),
            E("shadows", True),
            E("grid", False),
        ),
        E(
            "light",
            E("cast_shadows", True),
            pose(0, 0, 30, 0, 0, 0),
            E("diffuse", vec(0.9, 0.9, 0.9, 1.0)),
            E("specular", vec(0.25, 0.25, 0.25, 1.0)),
            E("attenuation",
              E("range", 1000.0), E("constant", 0.9),
              E("linear", 0.01), E("quadratic", 0.001)),
            E("direction", vec(-0.5, 0.35, -0.9)),
            name="sun", type="directional",
        ),
    ]


def auto_camera_pose(cfg, bbox) -> list:
    """A camera pose that frames the whole track.

    Sits due south of the track centre, looking north and down. Recomputed on
    every generation so it stays correct when the layout changes -- a hardcoded
    pose silently stops framing anything the moment a row gets longer.
    """
    x0, y0, x1, y1 = bbox
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    span = max(x1 - x0, y1 - y0)

    standoff = span * 0.62
    height = span * cfg["gui"]["camera_elevation"]
    camx, camy, camz = cx, cy - standoff, height

    # Camera looks along its own +X, so yaw points it at the centre and pitch
    # tilts it down by the angle subtended by the height over the ground run.
    yaw = math.atan2(cy - camy, cx - camx)
    pitch = math.atan2(camz, math.hypot(cx - camx, cy - camy))
    return [round(v, 3) for v in (camx, camy, camz, 0.0, pitch, yaw)]


def build_gui(cfg, bbox=None) -> E:
    if cfg["gui"].get("camera_auto", False) and bbox is not None:
        cam = auto_camera_pose(cfg, bbox)
    else:
        cam = cfg["gui"]["camera_pose"]
    plugins = [
        E("plugin",
          E("gz-gui",
            E("title", "3D View"),
            E("property", "false", type="bool", key="showTitleBar"),
            E("property", "docked", type="string", key="state")),
          E("engine", cfg["render"]["engine"]),
          E("scene", "scene"),
          E("ambient_light", vec(0.55, 0.55, 0.55)),
          E("background_color", vec(0.70, 0.78, 0.88)),
          E("camera_pose", vec(cam)),
          filename="MinimalScene", name="3D View"),
        E("plugin", filename="GzSceneManager", name="Scene Manager"),
        E("plugin", filename="InteractiveViewControl", name="Interactive view control"),
        E("plugin", filename="CameraTracking", name="Camera Tracking"),
        E("plugin", filename="MarkerManager", name="Marker manager"),
        E("plugin", filename="SelectEntities", name="Select Entities"),
        E("plugin", filename="Spawn", name="Spawn Entities"),
        E("plugin", filename="EntityContextMenuPlugin", name="Entity context menu"),
        E("plugin", filename="VisualizationCapabilities", name="Visualization Capabilities"),
        E("plugin",
          E("gz-gui",
            E("title", "World control"),
            E("property", "false", type="bool", key="showTitleBar"),
            E("property", "floating", type="string", key="state")),
          Comment("Pausing is controlled from the launch file (paused:=true), "
                  "not here, so the two cannot disagree."),
          E("play_pause", True), E("step", True), E("start_paused", False),
          filename="WorldControl", name="World control"),
        E("plugin",
          E("gz-gui",
            E("title", "World stats"),
            E("property", "false", type="bool", key="showTitleBar"),
            E("property", "floating", type="string", key="state")),
          E("sim_time", True), E("real_time", True), E("real_time_factor", True),
          filename="WorldStats", name="World stats"),
        E("plugin", filename="ComponentInspector", name="Component inspector"),
        E("plugin", filename="EntityTree", name="Entity tree"),
    ]
    return E("gui", *plugins, fullscreen="0")


def build_ground(cfg, bbox, holes=None) -> list:
    """Ground is a BOX, not a plane.

    A <plane> collision in Gazebo is mathematically infinite, so nothing could
    ever sit below z=0. The 40 cm water basin in Phase 4 needs exactly that, so
    the ground is boxes from the start and `holes` carves rectangles out of it.

    `bbox` is the track's extent plus margin. The ground is fitted to it rather
    than being centred on the world origin, because the origin is the START LINE
    (a useful spawn pose) and not the middle of the parkur.
    """
    g = cfg["ground"]
    th = g["thickness"]
    lip = cfg["track"]["surface_lip"]
    top = -lip                      # ground sits `lip` below the road surface
    holes = holes or []

    x0, y0, x1, y1 = bbox
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    sx = max(x1 - x0, g["min_size_x"])
    sy = max(y1 - y0, g["min_size_y"])
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0

    out = [banner("GROUND",
                  f"{sx:.1f} x {sy:.1f} m, centred on the track at "
                  f"({cx:.1f}, {cy:.1f}), not on the world origin.\n"
                  "Box, not plane: Phase 4 cuts a hole here for the water pit.\n"
                  f"Top face is {lip} m below the road, so the two never z-fight.")]

    if not holes:
        out.append(static_box("ground", (sx, sy, th), (cx, cy, top - th / 2.0),
                              GREY_GROUND, mu=g["friction"]))
        return out

    # Carve one rectangular hole into four surrounding tiles.
    if len(holes) > 1:
        raise NotImplementedError("multi-hole ground carving is Phase 4 work")
    hx0, hy0, hx1, hy1 = holes[0]
    tiles = [
        ("south", (x0, y0, x1, hy0)),
        ("north", (x0, hy1, x1, y1)),
        ("west", (x0, hy0, hx0, hy1)),
        ("east", (hx1, hy0, x1, hy1)),
    ]
    for name, (ax, ay, bx, by) in tiles:
        w, d = bx - ax, by - ay
        if w <= 0.001 or d <= 0.001:
            continue
        out.append(static_box(f"ground_{name}", (w, d, th),
                              ((ax + bx) / 2.0, (ay + by) / 2.0, top - th / 2.0),
                              GREY_GROUND, mu=g["friction"]))
    return out


# ---------------------------------------------------------------------------
#  Phase 2 - road surface and barriers
# ---------------------------------------------------------------------------

def _local_up(roll, pitch, yaw):
    """World direction of a box's own +Z after SDF's Rz(yaw)Ry(pitch)Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (cr * sp * cy + sr * sy,
            cr * sp * sy - sr * cy,
            cr * cp)


def _span_box(p0, p1, width, thickness, overlap, roll=0.0, sink=None):
    """Geometry for a box bridging two world points ON ITS TOP FACE.

    Used for both road slabs and barrier segments. Deriving the length and yaw
    from the two endpoints means straights and curves need no special cases:
    on a curve the outer barrier simply gets longer segments than the inner one,
    which is exactly right.

    `p0`/`p1` are points on the box's TOP face, and the centre is found by
    sinking half the thickness along the box's own down axis. Offsetting in
    world -Z instead would be wrong the moment the box is pitched or rolled:
    a 2 m thick slab on a 24 degree ramp would end up half a metre downrange of
    where its surface was supposed to be. Pass a negative `sink` to raise the
    box above the given points instead (used for barriers).
    """
    dx, dy, dz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
    horiz = math.hypot(dx, dy)
    length = math.sqrt(horiz * horiz + dz * dz)
    if length < 1e-9:
        return None
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, horiz)
    if sink is None:
        sink = thickness / 2.0
    ux, uy, uz = _local_up(roll, pitch, yaw)
    centre = ((p0[0] + p1[0]) / 2.0 - sink * ux,
              (p0[1] + p1[1]) / 2.0 - sink * uy,
              (p0[2] + p1[2]) / 2.0 - sink * uz)
    return dict(size=(length * overlap, width, thickness),
                centre=centre, yaw=yaw, pitch=pitch)


def slider_slot(cfg, centerline):
    """(s0, s1) of the barrier opening the kayar engel slides through.

    S6.1 says the barriers are continuous "virajlar dahil". S6.8 says the kayar
    engel must be able to leave the parkur completely. Both cannot hold at the
    same time, so the barrier gets one slot, no wider than the mechanism needs.
    """
    s = centerline.middle("s6_slider")
    half = cfg["slider"]["barrier_gap"] / 2.0
    return (s - half, s + half)


def build_track(cfg, centerline, gaps=()) -> list:
    """The 3 m road surface and the continuous barriers on both edges.

    S6.1: "Yol genisligi 3 metre olacaktir. Yol genisligi iki kenardan yol
    bariyerleri ile sinirlandirilacaktir. Yol kenarindaki bariyerler ... 80 cm
    +/- 10 cm yuksekliginde, yolun tamamini kapsayacak sekilde (virajlar dahil)
    surekli olacaktir."

    Everything is placed through centerline.frame_at(), so when Phase 3 adds the
    ramp heights and the yan egim bank to the profile, the road and both barrier
    lines ride over them without a line of code changing here.
    """
    t = cfg["track"]
    width = t["road_width"]
    th = t["surface_thickness"]
    overlap = t["overlap"]
    b_h, b_t = t["barrier_height"], t["barrier_thickness"]
    edge = width / 2.0 + b_t / 2.0

    road_links, barrier_links = [], []

    breaks = [b for g in gaps for b in g]
    steps = list(centerline.steps(t["tile_length"], t["barrier_arc_step_deg"],
                                  breakpoints=breaks))
    n_gapped = 0
    for i, (s0, s1, prim) in enumerate(steps):
        f0, f1 = centerline.frame_at(s0), centerline.frame_at(s1)
        mid_s = 0.5 * (s0 + s1)
        in_gap = any(g0 <= mid_s <= g1 for g0, g1 in gaps)

        # --- road slab: top face flush with the profile height --------------
        # Banked sections get a thicker slab. Tilting a 3 m road by 11.3 deg
        # lifts one edge 59 cm; with a thin slab the high side would hang in
        # mid-air over a visible void. Adding width x sin(roll) buries the whole
        # section and it reads as an embankment. Ramps stay thin on purpose --
        # a 20 cm deck looks like the ramp structure a real course would use.
        bank_fill = width * abs(math.sin(f0.roll))
        slab_th = th + bank_fill
        slab = _span_box(f0.offset(), f1.offset(), width, slab_th, overlap,
                         roll=f0.roll)
        if slab:
            road_links.append(E(
                "link",
                pose(*slab["centre"], f0.roll, slab["pitch"], slab["yaw"]),
                E("collision",
                  E("geometry", E("box", E("size", vec(slab["size"])))),
                  friction(1.0, 1.0),
                  name=f"road_{i:04d}_col"),
                E("visual",
                  E("geometry", E("box", E("size", vec(slab["size"])))),
                  material(GREY_RAMP if max(f0.z, f1.z) > 0.05 else GREY_ROAD),
                  name=f"road_{i:04d}_vis"),
                name=f"road_{i:04d}",
            ))

        # --- barriers: one run per side, centred b_h/2 above the road -------
        if in_gap:
            n_gapped += 1
            continue
        for side, lat in (("l", +edge), ("r", -edge)):
            # Negative sink raises the barrier above the road edge it stands on.
            seg = _span_box(f0.offset(lateral=lat), f1.offset(lateral=lat),
                            b_t, b_h, overlap, roll=f0.roll, sink=-b_h / 2.0)
            if not seg:
                continue
            # S6.1 red/white banding. One colour per segment, alternating along
            # the run, which is how the real barriers are painted -- and it
            # costs nothing, because a solid-colour material needs no texture.
            tint = BARRIER_RED if (i % 2 == 0) else BARRIER_WHITE
            barrier_links.append(E(
                "link",
                pose(*seg["centre"], f0.roll, seg["pitch"], seg["yaw"]),
                E("collision",
                  E("geometry", E("box", E("size", vec(seg["size"])))),
                  friction(0.6, 0.6),
                  name=f"barrier_{side}_{i:04d}_col"),
                E("visual",
                  E("geometry", E("box", E("size", vec(seg["size"])))),
                  material(tint),
                  name=f"barrier_{side}_{i:04d}_vis"),
                name=f"barrier_{side}_{i:04d}",
            ))

    return [
        banner("ROAD SURFACE",
               f"S6.1: {width} m wide. {len(road_links)} slabs along "
               f"{centerline.total_length:.1f} m of centerline."),
        E("model", E("static", True), pose(), *road_links, name="parkur_road"),
        banner("BARRIERS",
               f"S6.1: {b_h} m tall, continuous through the turns. "
               f"{len(barrier_links)} segments"
               + (f", with {n_gapped} omitted for the kayar engel slot."
                  if n_gapped else ".")),
        E("model", E("static", True), pose(), *barrier_links, name="parkur_barriers"),
    ]


# ---------------------------------------------------------------------------
#  Phase 3 - the elevation profile: %45 ramps and the %20 yan egim
# ---------------------------------------------------------------------------

def apply_profile(cfg, centerline) -> dict:
    """Write the ramps and the side slope into the centerline's Profile.

    This is the whole point of the Phase 2 architecture. Nothing here builds
    geometry -- it only says "the track is 2.25 m up at s=235.84" and "the track
    is banked 11.3 degrees from s=53 to s=67". The road slabs and both barrier
    runs are already placed through frame_at(), so they follow automatically.
    """
    prof = centerline.profile
    info = {}

    # -- S6.10: %45 climb, plateau, %45 descent ----------------------------
    grade = cfg["ramp"]["grade_percent"] / 100.0
    up0, up1 = centerline.span("s8_up")
    pl0, pl1 = centerline.span("s9_plat")
    dn0, dn1 = centerline.span("s10_down")

    # s is horizontal distance, so rise = horizontal run x grade. That makes the
    # grade exactly 45% as specified, and the travelled slope slightly longer.
    rise = (up1 - up0) * grade

    prof.add_height(0.0, 0.0)
    prof.add_height(up0, 0.0)
    prof.add_height(up1, rise)
    prof.add_height(pl0, rise)
    prof.add_height(pl1, rise)
    prof.add_height(dn1, 0.0)
    prof.add_height(centerline.total_length, 0.0)

    info["ramp_rise"] = rise
    info["ramp_angle_deg"] = math.degrees(math.atan(grade))
    info["slope_length"] = math.hypot(up1 - up0, rise)

    # -- S6.3: the water crossing is a dip, not a bump ----------------------
    wtr = cfg["water"]
    w0, w1 = centerline.span("s1_water")
    pool, bank = wtr["length"], wtr["bank_slope_length"]
    span = pool + 2 * bank
    mid = (w0 + w1) / 2.0
    d0 = mid - span / 2.0                      # water starts dropping
    d1 = mid + span / 2.0                      # back up to ground level
    depth = -wtr["depth"]

    prof.add_height(d0, 0.0)
    prof.add_height(d0 + bank, depth)
    prof.add_height(d1 - bank, depth)
    prof.add_height(d1, 0.0)

    info["water_span"] = (d0, d1)
    info["water_bank_pct"] = wtr["depth"] / bank * 100.0

    # -- S6.5: %20 side slope ----------------------------------------------
    ss = cfg["side_slope"]
    roll = math.atan(ss["grade_percent"] / 100.0)
    b0, b1 = centerline.span("s3_bank")
    t = ss["ramp_in_length"]

    prof.add_bank(b0, 0.0)
    prof.add_bank(b0 + t, roll)
    prof.add_bank(b1 - t, roll)
    prof.add_bank(b1, 0.0)

    # Banking tips one edge of the road below ground level. Lift the centerline
    # by exactly half the width x sin(roll) so the LOW edge stays at ground
    # level and the section reads as an embankment instead of a trench.
    lift = (cfg["track"]["road_width"] / 2.0) * math.sin(roll)
    prof.add_height(b0, 0.0)
    prof.add_height(b0 + t, lift)
    prof.add_height(b1 - t, lift)
    prof.add_height(b1, 0.0)

    info["bank_deg"] = math.degrees(roll)
    info["bank_lift"] = lift
    return info


# ---------------------------------------------------------------------------
#  Phase 3 - obstacle geometry
# ---------------------------------------------------------------------------

def _place(frame, lateral=0.0, normal=0.0, extra_yaw=0.0):
    """(x, y, z, roll, pitch, yaw) for something sitting on the road."""
    x, y, z = frame.offset(lateral=lateral, normal=normal)
    return (x, y, z, frame.roll, frame.pitch, frame.yaw + extra_yaw)


def _shape(name, size, rel_pose, rgba, mu=1.0):
    """A collision + visual pair, posed relative to the link origin."""
    geom = lambda: E("geometry", E("box", E("size", vec(size))))          # noqa: E731
    return [
        E("collision", pose(*rel_pose), geom(), friction(mu, mu),
          name=f"{name}_col"),
        E("visual", pose(*rel_pose), geom(), material(rgba), name=f"{name}_vis"),
    ]


def _bump_shapes(name, cfg):
    """A Sekil 4 trapezoid built from three boxes.

    A plain box would present a 5 cm vertical wall to the wheel; the sartname
    draws chamfers, and that difference is the whole point of the obstacle.
    Three primitives reproduce the cross-section exactly and avoid a mesh file,
    so there is no asset path that can fail to resolve at load time.
    """
    b = cfg["bumps"]
    h, span = b["height"], b["span"]
    half_b, half_t = b["base_width"] / 2.0, b["top_width"] / 2.0
    run = half_b - half_t                      # horizontal length of one chamfer
    slope_len = math.hypot(run, h)
    theta = math.atan2(h, run)                 # 55 deg at the Sekil 4 proportions
    thick = max(0.06, h * 1.2)                 # buried depth, keeps it solid

    out = list(_shape(f"{name}_top", (b["top_width"], span, h),
                      (0, 0, h / 2.0, 0, 0, 0), GREY_LIGHT))

    for sign, tag in ((-1.0, "in"), (+1.0, "out")):
        # Midpoint of the sloping face, then step down along the face normal by
        # half the box thickness to get the box centre.
        mx = sign * (half_b + half_t) / 2.0
        mz = h / 2.0
        pitch = -sign * theta
        nx, nz = -math.sin(pitch), math.cos(pitch)     # local +z in world terms
        cx, cz = mx - (thick / 2.0) * nx, mz - (thick / 2.0) * nz
        out += _shape(f"{name}_{tag}", (slope_len, span, thick),
                      (cx, 0, cz, 0, pitch, 0), GREY_LIGHT)
    return out


def _cone_boxes(cfg):
    """The S6.7 cone as (tag, size, z_centre, colour, mu) boxes, bottom up.

    Pulled out of _cone_shapes so the inertia calculation can read the SAME
    description the geometry is built from. A dynamic body whose inertia
    describes a different shape than its collisions tips at the wrong moment,
    and nothing in the SDF would show it.
    """
    c = cfg["cones"]
    base, height, tiers = c["base"], c["height"], c["tiers"]
    # The base plate grips; the plastic body above it does not. Ranking them
    # this way is what makes a clipped cone topple and stay put instead of
    # skating away down the road.
    out = [("base", (base, base, 0.04), 0.02, CONE_WHITE, 1.0)]
    tier_h = (height - 0.04) / tiers
    for k in range(tiers):
        f0 = k / tiers
        w = base * 0.78 * (1.0 - 0.82 * f0)
        z = 0.04 + tier_h * (k + 0.5)
        out.append((f"t{k}", (w, w, tier_h), z,
                    CONE_ORANGE if k % 2 == 0 else CONE_WHITE, 0.8))
    return out


def _cone_shapes(name, cfg):
    """S6.7 cone: a stack of tapering boxes on a square base."""
    out = []
    for tag, size, z, col, mu in _cone_boxes(cfg):
        out += _shape(f"{name}_{tag}", size, (0, 0, z, 0, 0, 0), col, mu=mu)
    return out


def _cone_inertial(cfg):
    """<inertial> for one cone: mass split by volume over the box stack.

    Each box contributes the solid-box formula about its own centre plus a
    parallel-axis term, which is exact for this shape and needs no mesh. The
    centre of mass comes out low (the base plate is the widest, densest slice),
    which is the whole point: S6.7 scores a cone TOUCH, so a brushed cone has
    to tip rather than stand there acting as a bollard.
    """
    boxes = _cone_boxes(cfg)
    mass = cfg["cones"]["mass"]
    vols = [w * d * h for _tag, (w, d, h), _z, _c, _mu in boxes]
    ms = [mass * v / sum(vols) for v in vols]
    com = sum(m * b[2] for m, b in zip(ms, boxes)) / mass

    ixx = iyy = izz = 0.0
    for m, (_tag, (w, d, h), z, _c, _mu) in zip(ms, boxes):
        dz = z - com
        ixx += m * (d * d + h * h) / 12.0 + m * dz * dz
        iyy += m * (w * w + h * h) / 12.0 + m * dz * dz
        izz += m * (w * w + d * d) / 12.0
    return E("inertial", pose(0, 0, com, 0, 0, 0), E("mass", mass),
             E("inertia", E("ixx", ixx), E("iyy", iyy), E("izz", izz),
               E("ixy", 0), E("ixz", 0), E("iyz", 0)))


def stop_line_stations(cfg, centerline):
    """(tag, s) for each S6.10 stop line, climb first.

    Shared by the painted marks in build_obstacles and the DUR/STOP plates in
    build_signage. Those are built in different functions from the same two
    numbers, and a plate standing a couple of metres from its own line is the
    kind of thing that looks fine in a screenshot and is wrong in the world --
    so there is one definition of where a stop line is, not two.
    """
    off = cfg["ramp"]["stop_line_offset"]
    out = []
    for seg, tag in (("s8_up", "cikis"), ("s10_down", "inis")):
        s0, s1 = centerline.span(seg)
        # Measured from the foot of the climb but from the TOP of the descent:
        # both marks are the same distance into the slope as the vehicle meets
        # it, which is what S7.5 is testing the brakes against.
        out.append((tag, s0 + off if tag == "cikis" else s1 - off))
    return out


def build_obstacles(cfg, centerline, rng) -> list:
    """Everything that physically changes how the vehicle moves.

    The ramps and the side slope are NOT here -- they live in the elevation
    profile and are already baked into the road surface. What is left is the
    things that sit ON the road.
    """
    out = []
    t = cfg["track"]
    road_w = t["road_width"]

    # -- S6.6: 15 cm dik engel ---------------------------------------------
    vb = cfg["vertical_block"]
    s = centerline.middle("s4_block")
    f = centerline.frame_at(s)
    out += [
        banner("S6.6  DIK ENGEL",
               f"{vb['height'] * 100:.0f} cm vertical block, full road width. "
               "S7.2 asks the drivetrain for 20 cm of instantaneous torque "
               "capability, but the obstacle itself is 15 cm."),
        E("model", E("static", True),
          pose(*_place(f, normal=vb["height"] / 2.0)),
          E("link",
            *_shape("block", (vb["depth"], road_w, vb["height"]),
                    (0, 0, 0, 0, 0, 0), GREY_DARK),
            name="block_link"),
          name="dik_engel"),
    ]

    # -- S6.7: trafik konileri ---------------------------------------------
    c = cfg["cones"]
    c0, c1 = centerline.span("s5_cones")
    c0, c1 = c0 + c["margin"], c1 - c["margin"]
    cone_models = []
    for i in range(c["count"]):
        frac = i / max(1, c["count"] - 1)
        s = c0 + frac * (c1 - c0) + rng.uniform(-c["jitter"], c["jitter"])
        lat = c["slalom_offset"] * (1 if i % 2 == 0 else -1)
        lat += rng.uniform(-c["jitter"], c["jitter"])
        f = centerline.frame_at(s)
        # A static cone is an immovable bollard: brush it and it deflects the
        # vehicle or catches a wheel. S6.7 scores a cone TOUCH at -5 points and
        # the real cone falls over, so a pinned one punishes contact harder
        # than the event does and trains the wrong avoidance behaviour. Set
        # cones.dynamic false to pin them again -- see config.yaml for when.
        cone_models.append(
            E("model", E("static", not c["dynamic"]),
              pose(*_place(f, lateral=lat)),
              E("link", _cone_inertial(cfg) if c["dynamic"] else None,
                *_cone_shapes(f"cone{i:02d}", cfg),
                name=f"cone{i:02d}_link"),
              name=f"trafik_konisi_{i:02d}"))
    out += [
        banner("S6.7  TRAFIK KONILERI",
               f"{c['count']} cones, {c['base']} m square base, "
               f"{c['height']} m tall, "
               + (f"{c['mass']} kg and free to topple"
                  if c["dynamic"] else "static") +
               ". S6.7 leaves placement to the judges on "
               "the day, so the slalom is seeded rather than fixed."),
        *cone_models,
    ]

    # -- S6.9: engebeli arazi ----------------------------------------------
    b = cfg["bumps"]
    b0, b1 = centerline.span("s7_bumps")
    b0, b1 = b0 + 1.5, b1 - 1.5
    bump_links = []
    n = 0
    for series, lat in (("L", +b["series_offset"]), ("R", -b["series_offset"])):
        phase = 0.0 if series == "L" or not b["stagger"] else b["row_spacing"] / 2.0
        s = b0 + phase
        k = 0
        while s <= b1 and k < b["rows"]:
            sj = s + rng.uniform(-b["jitter"], b["jitter"])
            f = centerline.frame_at(sj)
            bump_links.append(E(
                "link", pose(*_place(f, lateral=lat)),
                *_bump_shapes(f"bump_{series}{k:02d}", cfg),
                name=f"bump_{series}{k:02d}"))
            s += b["row_spacing"]
            k += 1
            n += 1
    out += [
        banner("S6.9  ENGEBELI ARAZI",
               f"{n} trapezoid bumps, {b['height'] * 100:.0f} cm tall on a "
               f"{b['base_width'] * 100:.0f} cm base, in two staggered series "
               f"{b['series_offset'] * 2:.1f} m apart. Separate series because "
               "S6.9 penalises taking the whole vehicle over one of them."),
        E("model", E("static", True), pose(), *bump_links, name="engebeli_arazi"),
    ]

    # -- S6.10: plinth under the firing plateau ----------------------------
    # The ramps are left as open decks, which is how a real course builds them.
    # A 6 m platform hanging 2.25 m in the air on a 20 cm deck is another
    # matter, so it gets a plinth. Purely cosmetic, but it also stops the
    # vehicle's downward-looking sensors seeing straight through the world.
    p0, p1 = centerline.span("s9_plat")
    fa, fb = centerline.frame_at(p0), centerline.frame_at(p1)
    plat_z = fa.z
    if plat_z > 0.3:
        seg = _span_box(fa.offset(normal=-t["surface_thickness"]),
                        fb.offset(normal=-t["surface_thickness"]),
                        road_w * 0.9, plat_z, t["overlap"])
        out += [
            banner("ATIS PLATEAU PLINTH",
                   f"Fill under the {plat_z:.2f} m firing platform."),
            E("model", E("static", True),
              pose(*seg["centre"], 0, 0, seg["yaw"]),
              E("link", *_shape("plinth", seg["size"], (0, 0, 0, 0, 0, 0),
                                GREY_RAMP),
                name="plinth_link"),
              name="atis_plinth"),
        ]

    # -- S6.10: stop markings on the ramps ---------------------------------
    r = cfg["ramp"]
    marks = []
    for tag, s in stop_line_stations(cfg, centerline):
        f = centerline.frame_at(s)
        marks.append(E(
            "link", pose(*_place(f, normal=0.006)),
            E("visual",
              E("geometry", E("box", E("size", vec(r["stop_line_width"],
                                                   road_w, 0.012)))),
              material(MARK_WHITE),
              name=f"stop_{tag}_vis"),
            name=f"stop_{tag}"))
    out += [
        banner("S6.10  STOP MARKINGS",
               "Painted ON the %45 slope, not on flat pads. S6.10 describes a "
               "continuous climb and descent and S7.5 requires the brakes to "
               "hold on exactly that grade, so flattening these would remove "
               "the thing being tested. Visual only, no collision."),
        E("model", E("static", True), pose(), *marks, name="stop_markings"),
    ]
    return out


def water_hole(cfg, centerline):
    """Axis-aligned rectangle the ground must NOT occupy.

    The basin only works if the ground has a hole in it: the road dips to
    -0.40 m but the ground box top is at -0.02, so a solid ground would simply
    fill the pit and the vehicle would drive straight over it.
    """
    d0, d1 = _water_span(cfg, centerline)
    half = (cfg["track"]["road_width"] / 2.0
            + cfg["track"]["barrier_thickness"]
            - cfg["water"]["wall_overlap"]
            + cfg["water"]["wall_thickness"])
    xs, ys = [], []
    n = 60
    for i in range(n + 1):
        f = centerline.frame_at(d0 + (d1 - d0) * i / n)
        for lat in (+half, -half):
            px, py, _ = f.offset(lateral=lat)
            xs.append(px)
            ys.append(py)
    return (min(xs), min(ys), max(xs), max(ys))


def _water_span(cfg, centerline):
    w0, w1 = centerline.span("s1_water")
    span = cfg["water"]["length"] + 2 * cfg["water"]["bank_slope_length"]
    mid = (w0 + w1) / 2.0
    return (mid - span / 2.0, mid + span / 2.0)


def build_water(cfg, centerline) -> list:
    """S6.3 su gecisi: a 40 cm basin, retaining walls, and buoyancy.

    Rain is omitted at the user's request; the depth is the part that decides
    whether a vehicle gets through, and it is the part S7.3 and S7.4 are written
    around (sealing, ground clearance, wheel diameter).

    The dip itself is already in the elevation profile, so the road and barriers
    descend into the pit on their own. What is left is: hold the surrounding
    earth back, and make water behave like water.
    """
    wtr = cfg["water"]
    t = cfg["track"]
    d0, d1 = _water_span(cfg, centerline)
    depth = wtr["depth"]
    out = [banner("S6.3  SU GECISI",
                  f"{depth * 100:.0f} cm deep, {wtr['length']:.1f} m pool with "
                  f"{wtr['bank_slope_length']:.1f} m banks "
                  f"({depth / wtr['bank_slope_length'] * 100:.0f} %). "
                  "Rain omitted by request.")]

    # -- retaining walls ---------------------------------------------------
    # These sit just outside the barrier line and hold up the ground that the
    # hole was cut from, so the pit has sides instead of an open void.
    inner = t["road_width"] / 2.0 + t["barrier_thickness"] - wtr["wall_overlap"]
    wall_c = inner + wtr["wall_thickness"] / 2.0
    wall_h = depth + 0.6                       # down past the pit floor
    wall_links = []
    steps = max(2, int(math.ceil((d1 - d0) / 2.0)))
    for i in range(steps):
        s0 = d0 + (d1 - d0) * i / steps
        s1 = d0 + (d1 - d0) * (i + 1) / steps
        f0, f1 = centerline.frame_at(s0), centerline.frame_at(s1)
        for side, lat in (("l", +wall_c), ("r", -wall_c)):
            # Anchored at ground level, hanging downwards. Deliberately NOT
            # following the road profile: this is the bank, not the road.
            a = f0.offset(lateral=lat)
            b = f1.offset(lateral=lat)
            a, b = (a[0], a[1], -t["surface_lip"]), (b[0], b[1], -t["surface_lip"])
            seg = _span_box(a, b, wtr["wall_thickness"], wall_h, t["overlap"])
            if seg:
                wall_links.append(E(
                    "link", pose(*seg["centre"], 0, 0, seg["yaw"]),
                    *_shape(f"wall_{side}{i:02d}", seg["size"],
                            (0, 0, 0, 0, 0, 0), GREY_DARK),
                    name=f"wall_{side}{i:02d}"))
    out.append(E("model", E("static", True), pose(), *wall_links,
                 name="su_havuzu_duvar"))

    # -- the water itself, visual only -------------------------------------
    # No collision. A collision box would just be a lid the vehicle drives on.
    fa, fb = centerline.frame_at(d0), centerline.frame_at(d1)
    surf_a = (fa.x, fa.y, 0.0)
    surf_b = (fb.x, fb.y, 0.0)
    body = _span_box(surf_a, surf_b, t["road_width"], depth, t["overlap"])
    out.append(E(
        "model", E("static", True),
        pose(*body["centre"], 0, 0, body["yaw"]),
        E("link",
          E("visual",
            E("geometry", E("box", E("size", vec(body["size"])))),
            E("transparency", 0.55),
            material(WATER_BLUE),
            name="water_vis"),
          name="water_link"),
        name="su_havuzu"))
    return out


def build_buoyancy(cfg) -> list:
    """Graded buoyancy with the interface at z = 0.

    Gazebo's Buoyancy system is a property of the WORLD, not of a volume, so
    there is no way to say "water lives only inside this box". That looked like
    a problem until the geometry solved it: the parkur surface never goes below
    z = 0 anywhere except inside this pit. Putting the air/water interface at
    z = 0 therefore makes the vehicle buoyant in exactly one place, for free,
    with no fluid simulation and no measurable cost.
    """
    if not cfg["water"]["enable_buoyancy"]:
        return [Comment("Buoyancy disabled via water.enable_buoyancy.")]
    return [
        banner("BUOYANCY",
               "Water below z=0, air above. The only part of the track below "
               "z=0 is the su gecisi basin, so this is water in the pit and "
               "nowhere else."),
        E("plugin",
          E("graded_buoyancy",
            E("default_density", cfg["water"]["fluid_density"]),
            E("density_change",
              E("above_depth", 0.0),
              E("density", 1.0))),
          filename="gz-sim-buoyancy-system",
          name="gz::sim::systems::Buoyancy"),
    ]


def build_slider(cfg, centerline) -> list:
    """S6.8 kayar engel: a 1 m blade crossing the road at 20 cm/s.

    Driven kinematically rather than by force. S6.8 specifies "20 cm/s hizla
    surekli rejimde" and "beklemeden ters yonde" -- a constant speed with an
    instant reversal. A PID chasing a triangle wave would sag on the straights
    and overshoot at the turnarounds; commanding joint velocity directly gives
    exactly the specified motion. The consequence is that the blade is
    immovable, so a vehicle that touches it gets shoved. Since contact is
    already a scored failure under S6.8, that seems like the right trade.
    """
    sl = cfg["slider"]
    t = cfg["track"]
    road_w = t["road_width"]

    s = centerline.middle("s6_slider")
    f = centerline.frame_at(s)
    amp = sl["travel"] / 2.0
    half_span = amp + sl["width"] / 2.0        # outermost edge of the blade
    post_y = half_span + sl["post_size"]

    blade_z = sl["ground_clearance"] + sl["height"] / 2.0
    m = sl["mass"]
    ix = m * (sl["width"] ** 2 + sl["height"] ** 2) / 12.0
    iy = m * (sl["thickness"] ** 2 + sl["height"] ** 2) / 12.0
    iz = m * (sl["thickness"] ** 2 + sl["width"] ** 2) / 12.0

    # Anchor sits outside the barrier on one side and is pinned to the world.
    # A model cannot be partly static, so this is how a fixed base is done.
    anchor = E(
        "link",
        pose(0, -post_y, 0.45, 0, 0, 0),
        E("inertial", E("mass", 50.0),
          E("inertia", E("ixx", 5), E("iyy", 5), E("izz", 5),
            E("ixy", 0), E("ixz", 0), E("iyz", 0))),
        *_shape("post_a", (sl["post_size"], sl["post_size"], 0.9),
                (0, 0, 0, 0, 0, 0), GREY_DARK),
        name="anchor",
    )

    blade = E(
        "link",
        pose(0, 0, blade_z, 0, 0, 0),
        E("inertial", E("mass", m),
          E("inertia", E("ixx", ix), E("iyy", iy), E("izz", iz),
            E("ixy", 0), E("ixz", 0), E("iyz", 0))),
        *_shape("blade", (sl["thickness"], sl["width"], sl["height"]),
                (0, 0, 0, 0, 0, 0), SLIDER_YELLOW, mu=0.6),
        name="blade",
    )

    # Visual-only rail and far post. No collision on the rail: it lies on the
    # road surface and a solid one would be an obstacle the sartname never
    # asked for.
    rail = E(
        "link",
        pose(0, 0, 0.01, 0, 0, 0),
        E("inertial", E("mass", 1.0),
          E("inertia", E("ixx", 1), E("iyy", 1), E("izz", 1),
            E("ixy", 0), E("ixz", 0), E("iyz", 0))),
        E("visual",
          E("geometry", E("box", E("size", vec(0.12, 2 * post_y, 0.02)))),
          material(GREY_DARK), name="rail_vis"),
        E("visual", pose(0, post_y, 0.44, 0, 0, 0),
          E("geometry", E("box", E("size", vec(sl["post_size"],
                                               sl["post_size"], 0.9)))),
          material(GREY_DARK), name="post_b_vis"),
        E("collision", pose(0, post_y, 0.44, 0, 0, 0),
          E("geometry", E("box", E("size", vec(sl["post_size"],
                                               sl["post_size"], 0.9)))),
          friction(0.6, 0.6), name="post_b_col"),
        name="rail",
    )

    limit = amp + 0.05        # never let the controller reach a hard stop
    model = E(
        "model",
        pose(*_place(f)),
        anchor,
        E("joint", E("parent", "world"), E("child", "anchor"),
          name="anchor_fixed", type="fixed"),
        rail,
        E("joint", E("parent", "anchor"), E("child", "rail"),
          name="rail_fixed", type="fixed"),
        blade,
        E("joint",
          E("parent", "anchor"),
          E("child", "blade"),
          E("axis",
            E("xyz", vec(0, 1, 0)),
            E("limit", E("lower", -limit), E("upper", limit),
              E("effort", 100000), E("velocity", 2.0)),
            E("dynamics", E("damping", 0.0), E("friction", 0.0))),
          name="slide", type="prismatic"),
        E("plugin",
          E("joint_name", "slide"),
          filename="gz-sim-joint-state-publisher-system",
          name="gz::sim::systems::JointStatePublisher"),
        E("plugin",
          E("joint_name", "slide"),
          Comment("use_force_commands false sets joint velocity directly, "
                  "which is what makes the 20 cm/s exact."),
          E("use_force_commands", False),
          E("topic", "/kayar_engel/cmd_vel"),
          filename="gz-sim-joint-controller-system",
          name="gz::sim::systems::JointController"),
        name="kayar_engel",
    )

    return [
        banner("S6.8  KAYAR ENGEL",
               f"{sl['width']:.1f} m blade, {sl['speed'] * 100:.0f} cm/s, "
               f"{sl['travel']:.1f} m peak to peak so it fully clears the "
               f"{road_w:.0f} m road on both sides. Motion comes from the "
               "kayar_engel_driver node; without it the blade sits still."),
        model,
    ]


def _billboard(name, mesh, width, height, position, face_yaw, thickness,
               backing=GREY_LIGHT, round_plate=False):
    """A textured mesh with a plain backing plate behind it.

    The mesh is modelled upright with its normal along -Y, so aiming it needs
    only a yaw of face_yaw + 90 degrees. `face_yaw` is the direction the printed
    side looks toward; the backing therefore ends up on the far side.

    A round plate gets a cylinder for its backing rather than a box, so the
    silhouette stays circular from every angle. The cylinder's axis is local Z,
    so roll of 90 degrees swings it onto the face normal.
    """
    yaw = face_yaw + math.pi / 2.0
    back_y = thickness / 2.0 + 0.004
    if round_plate:
        geom = E("cylinder", E("radius", width / 2.0), E("length", thickness))
        back_pose = pose(0, back_y, 0, math.pi / 2.0, 0, 0)
    else:
        geom = E("box", E("size", vec(width, thickness, height)))
        back_pose = pose(0, back_y, 0, 0, 0, 0)

    return E(
        "model", E("static", True), pose(*position, 0, 0, yaw),
        E("link",
          E("visual",
            E("geometry", E("mesh", E("uri", mesh))),
            name=f"{name}_face"),
          E("collision", back_pose, E("geometry", geom),
            friction(0.6, 0.6), name=f"{name}_col"),
          E("visual", back_pose, E("geometry", geom),
            material(backing), name=f"{name}_back"),
          name=f"{name}_link"),
        name=name,
    )


def _sign_post(cfg, name, link, x, y, road_z, yaw):
    """A post whose top lands signs.post_height above the road at `road_z`.

    The base always sits on the GROUND (-surface_lip), never on the road it
    stands beside. Those are the same surface on the flat rows but not on the
    S6.10 ramp, where the road climbs to 2.25 m while the verge next to it
    stays down: a post based on the road surface there stands on nothing.
    Only the length below the plate changes, so post_height keeps meaning
    "height above the road", which is what the camera sees.

    One helper for every post in the world -- stage tabelalar, drag strip and
    stop plates -- so that invariant cannot hold in two places and not a third.
    """
    s, lip = cfg["signs"], cfg["track"]["surface_lip"]
    length = road_z + lip + s["post_height"]
    geom = lambda: E("geometry", E("cylinder",              # noqa: E731
                                   E("radius", s["post_radius"]),
                                   E("length", length)))
    return E(
        "model", E("static", True), pose(x, y, -lip, 0, 0, yaw),
        E("link",
          E("collision", pose(0, 0, length / 2.0, 0, 0, 0), geom(),
            friction(0.6, 0.6), name=f"{link}_col"),
          E("visual", pose(0, 0, length / 2.0, 0, 0, 0), geom(),
            material(GREY_DARK), name=f"{link}_vis"),
          name=link),
        name=name)


def build_signage(cfg, centerline) -> list:
    """S6.2 numbered tabelalar and the S6.10 atis target.

    "Otonom gorevlerde yarismacilar bu tabelalari taniyarak ilgili parkura
     geldiklerini algilayacaktir."

    These two are the only textured objects in the world, because they are the
    only ones the vehicle has to READ rather than merely avoid.
    """
    s = cfg["signs"]
    t = cfg["track"]
    mesh_root = "model://ika_isaretler/meshes"
    out = []

    # -- stage tabelalar ---------------------------------------------------
    # Signs face back down the track at the approaching vehicle: the plate
    # normal is the reverse of the direction of travel.
    side = -1.0 if s["side"] == "right" else 1.0
    lat = side * (t["road_width"] / 2.0 + t["barrier_thickness"]
                  + s["lateral_offset"])
    sign_models = []
    plate = s["outer_diameter"]
    # The post has to stand BEHIND the plate, not through it. Both share the
    # same ground position, and the plate's printed face sits on that exact
    # plane -- so a post centred there comes out half in front of the number.
    # Shift it back by the plate's thickness plus its own radius, along the
    # direction of travel (which is away from the face).
    back = s["plate_thickness"] + s["post_radius"] + 0.01

    post_h = s["post_height"]
    for p in centerline.stages():
        # Placed at the START of the stage, not the middle. The vehicle has to
        # know it has arrived before the obstacle is under its wheels.
        f = centerline.frame_at(p.s0)
        px, py, pz = f.offset(lateral=lat)
        sign_models.append(_sign_post(
            cfg, f"tabela_direk_{p.stage:02d}", f"post{p.stage:02d}",
            px + back * math.cos(f.yaw), py + back * math.sin(f.yaw),
            pz, f.yaw))
        sign_models.append(_billboard(
            f"tabela_{p.stage:02d}",
            f"{mesh_root}/stage_{p.stage:02d}.obj",
            plate, plate,
            (px, py, pz + post_h), f.yaw + math.pi, s["plate_thickness"],
            round_plate=True))

    out += [
        banner("S6.2  ASAMA TABELALARI",
               f"{len(centerline.stages())} numbered signs, {s['outer_diameter']} m "
               "red ring, placed at the START of each stage so the vehicle can "
               "recognise the stage before it is in it. Textured because the "
               "autonomous run depends on reading them."),
        *sign_models,
    ]

    # -- S6.10 stop plates beside the painted stop lines -------------------
    # LEFT of travel, while every stage tabela is on the right. The side is
    # itself information: a plate on the right says "you are at stage n", a
    # plate on the left says "stop here". That is why this does not read
    # signs.side -- flipping the tabelalar to the left must not stack the two
    # families on the same verge.
    #
    # Position comes from stop_line_stations(), the same call the paint uses,
    # so the plate cannot end up beside the wrong part of the slope.
    stop_lat = (t["road_width"] / 2.0 + t["barrier_thickness"]
                + s["lateral_offset"])
    stop_models = []
    words = []
    for tag, station in stop_line_stations(cfg, centerline):
        f = centerline.frame_at(station)
        px, py, pz = f.offset(lateral=stop_lat)
        stop_models.append(_sign_post(
            cfg, f"dur_direk_{tag}", f"dur_post_{tag}",
            px + back * math.cos(f.yaw), py + back * math.sin(f.yaw),
            pz, f.yaw))
        stop_models.append(_billboard(
            f"dur_tabela_{tag}", f"{mesh_root}/dur_{tag}.obj", plate, plate,
            (px, py, pz + post_h), f.yaw + math.pi, s["plate_thickness"],
            round_plate=True))
        words.append(cfg["ramp"]["stop_sign_climb"] if tag == "cikis"
                     else cfg["ramp"]["stop_sign_descent"])
    out += [
        banner("S6.10  DUR TABELALARI",
               f"'{words[0]}' on the climb and '{words[1]}' on the descent, "
               "standing on the LEFT of travel beside the painted line the "
               "vehicle has to hold on for 2 s. Round plates like the "
               "tabelalar but inverted -- white on red -- so a stop "
               "instruction can never be read as a stage number."),
        *stop_models,
    ]

    # -- atis target -------------------------------------------------------
    tg = cfg["target"]
    plat_s = centerline.span("s9_plat")[1]
    plat_z = centerline.frame_at(plat_s).z
    end = centerline.frame_at(centerline.total_length)
    tx = end.x + tg["gap_beyond_road"] * math.cos(end.yaw)
    ty = end.y + tg["gap_beyond_road"] * math.sin(end.yaw)
    # Sekil 1 puts the target on the road axis just past the bitis line, which
    # is the only arrangement that makes the S6.10 range a property of the
    # layout rather than a free parameter -- so there is no lateral variant to
    # choose between. `level_with_vehicle` stays because it records WHY the
    # centre is where it is, but false has no defined height to fall back to,
    # and a target at the wrong height is a scoring difference nobody would
    # spot in the SDF. Stop instead.
    if not tg["level_with_vehicle"]:
        raise SystemExit(
            "config: target.level_with_vehicle must be true -- the atis "
            "geometry assumes a horizontal shot from the plateau (S6.10).")
    tz = plat_z + tg["aim_height"]

    frame_w = tg["paper_w"] + 2 * tg["frame_margin"]
    frame_h = tg["paper_h"] + 2 * tg["frame_margin"]
    out += [
        banner("S6.10  ATIS HEDEFI",
               f"A3 landscape, {tg['ring_inner_d'] * 100:.0f} / "
               f"{tg['ring_mid_d'] * 100:.0f} / {tg['ring_outer_d'] * 100:.0f} cm "
               f"rings, centre at z={tz:.2f} m which is plateau height plus "
               f"{tg['aim_height']:.2f} m, so the shot is horizontal."),
        # Support post, from the ground up to the bottom of the frame.
        E("model", E("static", True), pose(tx, ty, 0, 0, 0, end.yaw),
          E("link",
            E("collision", pose(0, 0, (tz - frame_h / 2.0) / 2.0, 0, 0, 0),
              E("geometry", E("cylinder", E("radius", tg["post_radius"]),
                              E("length", max(0.1, tz - frame_h / 2.0)))),
              friction(0.6, 0.6), name="hedef_post_col"),
            E("visual", pose(0, 0, (tz - frame_h / 2.0) / 2.0, 0, 0, 0),
              E("geometry", E("cylinder", E("radius", tg["post_radius"]),
                              E("length", max(0.1, tz - frame_h / 2.0)))),
              material(GREY_DARK), name="hedef_post_vis"),
            name="hedef_post"),
          name="hedef_direk"),
        _billboard("hedef", f"{mesh_root}/hedef.obj",
                   frame_w, frame_h, (tx, ty, tz),
                   end.yaw + math.pi, 0.04, backing=GREY_DARK),
    ]
    return out


def accel_geometry(cfg, centerline):
    """Where the hizlanma parkuru sits. Returns a dict of world coordinates.

    S6.11 puts it outside the main parkur -- "Hizlanma Parkuru Sekil-1'de
    goruldugu uzere ana parkur disinda konumlanacaktir" -- so it is a separate
    piece of geometry that shares nothing with the centerline.
    """
    a = cfg["accel_strip"]
    total = a["length"] + a["stop_zone"]
    width = a["lanes"] * a["lane_width"]

    # Centre it on the main track's x extent, and place it north of the
    # northernmost row.
    xs = [centerline.frame_at(centerline.total_length * i / 200).x
          for i in range(201)]
    ys = [centerline.frame_at(centerline.total_length * i / 200).y
          for i in range(201)]
    cx = (min(xs) + max(xs)) / 2.0
    y0 = max(ys) + a["offset_y"]

    return dict(
        x_start=cx - total / 2.0,                  # start line
        x_finish=cx - total / 2.0 + a["length"],   # 30 m mark
        x_end=cx + total / 2.0,                    # end of the stop zone
        y_south=y0, y_north=y0 + width,
        y_centre=y0 + width / 2.0,
        width=width, total=total,
    )


def build_accel_strip(cfg, centerline) -> list:
    """S6.11 hizlanma parkuru: 30 m from a standing start, then 10 m to stop.

    Straight, flat, and separate from the main parkur. The only things that
    matter here are the 30 m mark and the 10 m of run-off after it, because
    S6.11 penalises anyone who cannot stop within it.
    """
    a = cfg["accel_strip"]
    t = cfg["track"]
    s = cfg["signs"]
    g = accel_geometry(cfg, centerline)
    th = t["surface_thickness"]

    out = [banner("S6.11  HIZLANMA PARKURU",
                  f"{a['lanes']} lanes x {a['lane_width']:.1f} m, "
                  f"{a['length']:.0f} m timed then {a['stop_zone']:.0f} m to "
                  "stop. Outside the main parkur, per S6.11.")]

    # -- running surface, split so the stop zone reads differently ----------
    for name, x0, x1, col in (
            ("hizlanma_pist", g["x_start"], g["x_finish"], GREY_ROAD),
            ("hizlanma_durma", g["x_finish"], g["x_end"], GREY_RAMP)):
        out.append(static_box(
            name, (x1 - x0, g["width"], th),
            ((x0 + x1) / 2.0, g["y_centre"], -th / 2.0), col))

    # -- outer barriers, same spec as the main track -----------------------
    b_h, b_t = t["barrier_height"], t["barrier_thickness"]
    bar = []
    seg = 4.0
    n = int(math.ceil(g["total"] / seg))
    for i in range(n):
        x0 = g["x_start"] + g["total"] * i / n
        x1 = g["x_start"] + g["total"] * (i + 1) / n
        for side, y in (("s", g["y_south"] - b_t / 2.0),
                        ("n", g["y_north"] + b_t / 2.0)):
            # Same banding as the main track: S6.1 describes the parkur's
            # barriers, and the hizlanma parkuru is part of the parkur.
            tint = BARRIER_RED if i % 2 == 0 else BARRIER_WHITE
            bar.append(E(
                "link", pose((x0 + x1) / 2.0, y, b_h / 2.0, 0, 0, 0),
                *_shape(f"ab_{side}{i:02d}",
                        ((x1 - x0) * t["overlap"], b_t, b_h),
                        (0, 0, 0, 0, 0, 0), tint),
                name=f"ab_{side}{i:02d}"))
    out.append(E("model", E("static", True), pose(), *bar,
                 name="hizlanma_bariyer"))

    # -- painted lines: start, the 30 m mark, and the lane splits ----------
    marks = []
    for tag, x in (("baslangic", g["x_start"]), ("bitis", g["x_finish"])):
        marks.append(E(
            "link", pose(x, g["y_centre"], 0.006, 0, 0, 0),
            E("visual",
              E("geometry", E("box", E("size", vec(a["line_width"],
                                                   g["width"], 0.012)))),
              material(MARK_WHITE), name=f"line_{tag}_vis"),
            name=f"line_{tag}"))
    for k in range(1, a["lanes"]):
        y = g["y_south"] + k * a["lane_width"]
        marks.append(E(
            "link", pose((g["x_start"] + g["x_end"]) / 2.0, y, 0.006, 0, 0, 0),
            E("visual",
              E("geometry", E("box", E("size", vec(g["total"], 0.08, 0.012)))),
              material(MARK_WHITE), name=f"lane_{k}_vis"),
            name=f"lane_{k}"))
    out.append(E("model", E("static", True), pose(), *marks,
                 name="hizlanma_cizgiler"))

    # -- tabelalar at both ends --------------------------------------------
    # Vehicles run west to east here, so the signs face west.
    #
    # BOTH ends carry the same number. The finish plate is that number struck
    # through with a red diagonal, cancelling it the way an end-of-restriction
    # sign cancels the one that opened the section -- it is not a separate
    # twelfth sign. So the detector only ever has to answer "is this number
    # cancelled or not", which is a far easier question than telling an 11 from
    # a 12 at range.
    mesh_root = "model://ika_isaretler/meshes"
    num = a["sign_start"]
    if num > s["count"]:
        raise SystemExit(
            f"config: accel_strip.sign_start is {num} but signs.count is "
            f"{s['count']}, so stage_{num:02d} is never rendered. Raise "
            "signs.count to at least the drag strip's number.")
    for tag, x, mesh in (("baslangic", g["x_start"], f"stage_{num:02d}.obj"),
                         ("bitis", g["x_finish"], f"stage_{num:02d}_bitis.obj")):
        # NORTH verge, which is the vehicle's left on this eastbound run. That
        # is the side Sekil 1 draws both plates on -- the drag strip is a
        # separate piece of road, so it does not inherit the main track's
        # right-hand convention.
        y = g["y_north"] + b_t + s["lateral_offset"]
        back = s["plate_thickness"] + s["post_radius"] + 0.01
        out.append(_sign_post(cfg, f"hizlanma_direk_{tag}", f"apost_{tag}",
                              x + back, y, 0.0, 0.0))
        out.append(_billboard(
            f"hizlanma_tabela_{tag}", f"{mesh_root}/{mesh}",
            s["outer_diameter"], s["outer_diameter"],
            (x, y, s["post_height"]), math.pi, s["plate_thickness"],
            round_plate=True))
    return out


# ---------------------------------------------------------------------------
#  Stage map - a machine-readable index of where every stage lives
# ---------------------------------------------------------------------------

def stage_map(cfg, centerline) -> dict:
    """Emitted alongside the world as ika_parkur_stages.yaml.

    Phase 3 uses it to place obstacles, but it is just as useful to your
    autonomy stack: it is the ground truth for "stage 5 starts at this world
    coordinate", which makes it easy to score a run or to check whether the
    sign detector fired at the right place.
    """
    out = {"total_length": round(centerline.total_length, 3), "segments": []}
    for p in centerline.primitives:
        f0 = centerline.frame_at(p.s0)
        fm = centerline.frame_at(p.s0 + p.length / 2.0)
        f1 = centerline.frame_at(p.s0 + p.length)
        out["segments"].append({
            "id": p.id,
            "label": p.label,
            "stage": p.stage,
            "kind": p.kind,
            "s_start": round(p.s0, 3),
            "s_end": round(p.s0 + p.length, 3),
            "length": round(p.length, 3),
            "start_xy": [round(f0.x, 3), round(f0.y, 3), round(f0.z, 3)],
            "mid_xy": [round(fm.x, 3), round(fm.y, 3), round(fm.z, 3)],
            "end_xy": [round(f1.x, 3), round(f1.y, 3), round(f1.z, 3)],
            "heading_deg": round(math.degrees(fm.yaw) % 360.0, 2),
        })
    return out


def check_target_geometry(cfg, centerline) -> list:
    """Verify the atis distance implied by the layout still clears S6.10.

    The target is not built until Phase 6, but its distance is decided here, by
    the length of the descent ramp and the bitis straight. Checking it now means
    an innocent-looking edit to `finish` cannot silently produce an illegal shot.
    """
    tgt, ramp = cfg["target"], cfg["ramp"]
    problems = []

    fire_s = centerline.span("s9_plat")[1]        # leading edge of the plateau
    road_end_s = centerline.total_length
    actual = (road_end_s - fire_s) + tgt["gap_beyond_road"]

    if abs(actual - tgt["distance"]) > 0.05:
        problems.append(
            f"target.distance says {tgt['distance']} m but the layout gives "
            f"{actual:.2f} m (descent {ramp['run_length']} + bitis "
            f"{centerline.segment('finish').length} + gap {tgt['gap_beyond_road']})")
    if actual < 10.0:
        problems.append(
            f"atis distance is {actual:.2f} m, under the S6.10 minimum of 10 m. "
            f"Lengthen layout 'finish' by {10.0 - actual:.2f} m.")
    return problems, actual


def track_bounds(cfg, centerline, margin=None):
    """Axis-aligned extent of the track plus a margin.

    Drives both the ground size and the default camera pose, so the world
    reshapes itself around whatever the layout happens to be.
    """
    if margin is None:
        margin = cfg["ground"]["margin"]
    half = cfg["track"]["road_width"] / 2.0 + cfg["track"]["barrier_thickness"]
    xs, ys = [], []
    n = 400
    for i in range(n + 1):
        f = centerline.frame_at(centerline.total_length * i / n)
        for lat in (+half, -half):
            px, py, _ = f.offset(lateral=lat)
            xs.append(px)
            ys.append(py)
    # The hizlanma parkuru sits outside the centerline entirely, so its corners
    # have to be folded in or the ground will not reach under it.
    g = accel_geometry(cfg, centerline)
    xs += [g["x_start"], g["x_end"]]
    ys += [g["y_south"] - 2.0, g["y_north"] + 2.0]

    return (min(xs) - margin, min(ys) - margin,
            max(xs) + margin, max(ys) + margin)


# ---------------------------------------------------------------------------
#  Assembly
# ---------------------------------------------------------------------------

def build_world(cfg, centerline, rng, bbox) -> E:
    world = E("world", name=cfg["meta"]["world_name"])
    world.add(
        Comment(
            "TEKNOFEST 2026 - Insansiz Kara Araci Yarismasi parkuru\n"
            f"Generated from sartname v{cfg['meta']['sartname_version']} by "
            "generator/generate.py -- DO NOT EDIT BY HAND.\n"
            "Change generator/config.yaml and re-run the generator instead."
        )
    )
    world.add(build_physics(cfg))
    world.add(build_buoyancy(cfg))
    world.add(build_scene_and_light())
    world.add(build_gui(cfg, bbox))
    world.add(build_ground(cfg, bbox, holes=[water_hole(cfg, centerline)]))
    world.add(build_track(cfg, centerline, gaps=[slider_slot(cfg, centerline)]))
    world.add(build_obstacles(cfg, centerline, rng))
    world.add(build_water(cfg, centerline))
    world.add(build_slider(cfg, centerline))
    world.add(build_signage(cfg, centerline))
    world.add(build_accel_strip(cfg, centerline))
    return world


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--config", type=pathlib.Path, default=DEFAULT_CONFIG)
    ap.add_argument("-o", "--out", type=pathlib.Path, default=DEFAULT_OUT)
    ap.add_argument("-s", "--seed", type=int, default=None,
                    help="override random.seed from config (cone / bump layout)")
    ap.add_argument("--textures", action="store_true",
                    help="re-render the sign and target textures even if they "
                         "already exist (needed after changing signs.* or "
                         "target.* in config.yaml)")
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    seed = args.seed if args.seed is not None else cfg["random"]["seed"]
    rng = random.Random(seed)

    layout, pad_report = resolve_pads(cfg["layout"], cfg["track"]["row_length"])
    centerline = build_from_layout(layout, cfg["track"]["turn_radius"],
                                   profile=Profile())

    # The Profile is mutable and the Centerline holds a reference to it, so it
    # can be filled in after construction -- which it has to be, because the
    # ramp keyframes are addressed by segment name and those only exist once
    # the centerline has been walked.
    prof_info = apply_profile(cfg, centerline)

    # Textures before geometry: the world references mesh files by path, and a
    # missing OBJ turns into a silently invisible sign at load time.
    sign_report = signage.generate(cfg, HERE.parent / "models",
                                   force=args.textures)

    bbox = track_bounds(cfg, centerline)
    world = build_world(cfg, centerline, rng, bbox)
    text = sdf_document(world)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")

    smap = stage_map(cfg, centerline)
    map_path = args.out.with_name(args.out.stem + "_stages.yaml")
    map_path.write_text(
        "# Generated by generator/generate.py - where every stage lives.\n"
        "# Useful for scoring a run and for checking sign-detection timing.\n"
        + yaml.safe_dump(smap, sort_keys=False, default_flow_style=None),
        encoding="utf-8")

    x0, y0, x1, y1 = bbox
    need_x, need_y = x1 - x0, y1 - y0
    gx = max(need_x, cfg["ground"]["min_size_x"])
    gy = max(need_y, cfg["ground"]["min_size_y"])

    print(f"wrote {args.out}")
    print(f"      {map_path}")
    print(f"  seed        : {seed}")
    print(f"  models      : {text.count('<model ')}")
    print(f"  links       : {text.count('<link ')}")
    print(f"  centerline  : {centerline.total_length:.2f} m, "
          f"{len(centerline.stages())} numbered stages")
    for row, (fixed, each) in sorted(pad_report.items()):
        print(f"  row {row}       : {cfg['track']['row_length']:.1f} m "
              f"= {fixed:.1f} fixed + padding {each:.2f} m each")
    print(f"  ramps       : {prof_info['ramp_angle_deg']:.2f} deg, rise "
          f"{prof_info['ramp_rise']:.2f} m, slope length "
          f"{prof_info['slope_length']:.2f} m")
    print(f"  yan egim    : {prof_info['bank_deg']:.2f} deg, centreline lifted "
          f"{prof_info['bank_lift']:.3f} m so the low edge stays at ground")
    print(f"  track bbox  : x [{x0:.1f} .. {x1:.1f}]  y [{y0:.1f} .. {y1:.1f}]"
          f"   ({need_x:.1f} x {need_y:.1f} m)")
    print(f"  ground      : {gx:.1f} x {gy:.1f} m centred on "
          f"({(x0 + x1) / 2:.1f}, {(y0 + y1) / 2:.1f})")
    if cfg["gui"].get("camera_auto"):
        cam = auto_camera_pose(cfg, bbox)
        print(f"  gui camera  : {cam[0]:.1f} {cam[1]:.1f} {cam[2]:.1f}  "
              f"pitch {math.degrees(cam[4]):.0f} deg")

    if sign_report["written"]:
        font = pathlib.Path(sign_report["font"]).name
        print(f"  textures    : {sign_report['written']} written, font {font}")
        if not sign_report["arial"]:
            print("                note: S6.2 asks for Arial Black, which is "
                  "not installed. Using the heaviest available substitute.")
            print("                sudo apt install ttf-mscorefonts-installer")
    else:
        print(f"  textures    : {sign_report['skipped']} reused "
              "(pass --textures to re-render)")

    sl = cfg["slider"]
    need = cfg["track"]["road_width"] + sl["width"]
    print(f"  kayar engel : {sl['width']:.1f} m blade, {sl['travel']:.2f} m "
          f"peak to peak, {sl['travel'] / sl['speed'] / 2:.0f} s per sweep")

    problems, atis = check_target_geometry(cfg, centerline)
    if sl["travel"] < need - 1e-9:
        problems.append(
            f"slider.travel is {sl['travel']:.2f} m. S6.8 requires the blade to "
            f"leave the parkur completely, which needs road_width + width = "
            f"{need:.2f} m.")
    print(f"  atis range  : {atis:.2f} m from the plateau edge  "
          f"(S6.10 minimum 10 m)")

    for p in problems:
        print(f"  WARNING: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
