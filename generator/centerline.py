"""
Track centerline geometry.

The whole parkur is described as an ordered list of straights and arcs. Walking
that list gives a single arc-length coordinate `s` that runs from 0 at the start
line to `total_length` at the finish. Any point on the track can then be asked
for as `centerline.frame_at(s)`, which returns a full 3D frame: position plus
yaw, pitch and roll.

That one abstraction is what makes the later phases cheap. Placing the dik engel
becomes "put a box at s = 118.0" and it lands correctly whether that spot is on
a straight, inside a turn, or halfway up a 45% ramp -- and the road surface and
barriers automatically follow the same profile.

Frame convention (standard ROS / Gazebo):
    x east, y north, z up
    yaw   rotation about +z, 0 = heading east
    pitch nose-down positive
    roll  about the forward axis, positive banks the LEFT side upward
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Frame:
    """A pose on (or offset from) the centerline."""
    s: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def left(self) -> tuple:
        """Unit vector pointing to the left edge of the road, honouring bank."""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        return (-cr * math.sin(self.yaw), cr * math.cos(self.yaw), sr)

    def up(self) -> tuple:
        """Unit vector normal to the road surface, honouring bank."""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        return (sr * math.sin(self.yaw), -sr * math.cos(self.yaw), cr)

    def offset(self, lateral: float = 0.0, normal: float = 0.0) -> tuple:
        """World point `lateral` m to the left and `normal` m above the road."""
        lx, ly, lz = self.left()
        ux, uy, uz = self.up()
        return (self.x + lateral * lx + normal * ux,
                self.y + lateral * ly + normal * uy,
                self.z + lateral * lz + normal * uz)


# ---------------------------------------------------------------------------
#  Primitives
# ---------------------------------------------------------------------------

@dataclass
class Straight:
    length: float
    id: str = ""
    label: str = ""
    stage: int | None = None
    s0: float = 0.0

    kind = "straight"

    def walk(self, x, y, yaw):
        return (x + self.length * math.cos(yaw),
                y + self.length * math.sin(yaw),
                yaw)

    def point(self, t, x0, y0, yaw0):
        """t in [0, length] measured from the primitive's own start."""
        return (x0 + t * math.cos(yaw0), y0 + t * math.sin(yaw0), yaw0)


@dataclass
class Arc:
    radius: float
    sweep_deg: float
    turn: str          # "left" (counter-clockwise) or "right" (clockwise)
    id: str = ""
    label: str = ""
    stage: int | None = None
    s0: float = 0.0

    kind = "arc"

    def __post_init__(self):
        if self.turn not in ("left", "right"):
            raise ValueError(f"arc turn must be 'left' or 'right', got {self.turn!r}")
        self.length = self.radius * math.radians(self.sweep_deg)

    def _centre(self, x0, y0, yaw0):
        # Turn centre sits one radius to the inside of the turn.
        sign = 1.0 if self.turn == "left" else -1.0
        return (x0 - sign * self.radius * math.sin(yaw0),
                y0 + sign * self.radius * math.cos(yaw0))

    def point(self, t, x0, y0, yaw0):
        sign = 1.0 if self.turn == "left" else -1.0
        cx, cy = self._centre(x0, y0, yaw0)
        theta0 = math.atan2(y0 - cy, x0 - cx)
        dtheta = sign * (t / self.radius)
        theta = theta0 + dtheta
        return (cx + self.radius * math.cos(theta),
                cy + self.radius * math.sin(theta),
                yaw0 + dtheta)

    def walk(self, x, y, yaw):
        return self.point(self.length, x, y, yaw)


# ---------------------------------------------------------------------------
#  Elevation and bank profile
# ---------------------------------------------------------------------------

def _interp(keys, s: float) -> float:
    """Piecewise-linear lookup over a sorted list of (s, value) keyframes."""
    if not keys:
        return 0.0
    if s <= keys[0][0]:
        return keys[0][1]
    if s >= keys[-1][0]:
        return keys[-1][1]
    for (sa, va), (sb, vb) in zip(keys, keys[1:]):
        if sa <= s <= sb:
            if sb - sa < 1e-9:
                return vb
            return va + (s - sa) / (sb - sa) * (vb - va)
    return keys[-1][1]


class Profile:
    """Height and bank along the track.

    Both are keyframe lists interpolated linearly, which is what makes the %45
    ramps and the %20 yan egim work: the road slabs and both barrier lines are
    all placed through `frame_at`, so filling this in is enough to make the
    entire track ride over them. Nothing in the road builder knows a ramp exists.

    Bank uses keyframes rather than constant bands on purpose. A band would step
    the road from level to 11.3 degrees between one slab and the next, which is
    a cliff the vehicle would catch on. Keyframes let the bank wind on and off.
    """

    def __init__(self):
        self.heights: list[tuple[float, float]] = []   # (s, z)
        self.banks: list[tuple[float, float]] = []     # (s, roll_rad)

    def add_height(self, s: float, z: float) -> None:
        self.heights.append((s, z))
        self.heights.sort(key=lambda kv: kv[0])

    def add_bank(self, s: float, roll_rad: float) -> None:
        self.banks.append((s, roll_rad))
        self.banks.sort(key=lambda kv: kv[0])

    def height_at(self, s: float) -> float:
        return _interp(self.heights, s)

    def bank_at(self, s: float) -> float:
        return _interp(self.banks, s)


# ---------------------------------------------------------------------------
#  Centerline
# ---------------------------------------------------------------------------

class Centerline:
    """An ordered chain of primitives, addressable by arc length."""

    def __init__(self, primitives, profile: Profile | None = None,
                 origin=(0.0, 0.0), heading_deg: float = 180.0):
        self.primitives = list(primitives)
        self.profile = profile or Profile()

        # Pre-walk so every primitive knows its start pose and start arc length.
        x, y, yaw = origin[0], origin[1], math.radians(heading_deg)
        s = 0.0
        self._starts = []
        for p in self.primitives:
            p.s0 = s
            self._starts.append((x, y, yaw))
            x, y, yaw = p.walk(x, y, yaw)
            s += p.length
        self.total_length = s
        self.end_pose = (x, y, yaw)

    # -- lookup ------------------------------------------------------------
    def _locate(self, s: float):
        s = max(0.0, min(s, self.total_length))
        for p, start in zip(self.primitives, self._starts):
            if s <= p.s0 + p.length + 1e-9:
                return p, start, s - p.s0
        p = self.primitives[-1]
        return p, self._starts[-1], p.length

    def frame_at(self, s: float) -> Frame:
        p, (x0, y0, yaw0), t = self._locate(s)
        x, y, yaw = p.point(t, x0, y0, yaw0)
        z = self.profile.height_at(s)
        roll = self.profile.bank_at(s)

        # Pitch from the local slope of the height profile. Central difference,
        # clamped to the track ends.
        d = 0.25
        sa, sb = max(0.0, s - d), min(self.total_length, s + d)
        run = sb - sa
        pitch = 0.0
        if run > 1e-6:
            rise = self.profile.height_at(sb) - self.profile.height_at(sa)
            pitch = -math.atan2(rise, run)

        return Frame(s=s, x=x, y=y, z=z, yaw=yaw, pitch=pitch, roll=roll)

    # -- segment queries ---------------------------------------------------
    def segment(self, seg_id: str):
        for p in self.primitives:
            if p.id == seg_id:
                return p
        raise KeyError(f"no track segment with id {seg_id!r}")

    def span(self, seg_id: str) -> tuple:
        """(s_start, s_end) of a named segment."""
        p = self.segment(seg_id)
        return (p.s0, p.s0 + p.length)

    def middle(self, seg_id: str) -> float:
        p = self.segment(seg_id)
        return p.s0 + p.length / 2.0

    def stages(self):
        """Every primitive that carries a stage number, in order."""
        return [p for p in self.primitives if p.stage is not None]

    # -- sampling ----------------------------------------------------------
    def steps(self, step_straight: float, step_arc_deg: float,
              roll_tol_deg: float = 0.5, pitch_tol_deg: float = 1.0,
              min_step: float = 0.2, breakpoints=()):
        """Yield (s0, s1, primitive) intervals tiling the whole track.

        Straights get coarse steps, arcs get fine angular ones. Returning
        intervals rather than points lets callers build a box between each
        pair, which handles straights and curves with the same code.

        Intervals are then refined wherever the profile twists. Each tile is a
        rigid box carrying ONE roll and pitch, so a coarse tile spanning the
        start of the yan egim would jump from level to 11.3 degrees in a single
        step -- a lip across the road, exactly where the vehicle is most likely
        to be unsettled. Bisecting until the twist per tile is under tolerance
        turns that into a smooth wind-on.
        """
        rt, pt = math.radians(roll_tol_deg), math.radians(pitch_tol_deg)
        bps = sorted(breakpoints)
        for p in self.primitives:
            if p.kind == "arc":
                step = p.radius * math.radians(step_arc_deg)
            else:
                step = step_straight
            n = max(1, int(math.ceil(p.length / step - 1e-9)))

            # Breakpoints force a tile edge at a given s. Without them, cutting
            # the barrier slot for the kayar engel would mean deleting whichever
            # 4 m tile happened to contain it.
            edges = [p.s0 + i * p.length / n for i in range(n + 1)]
            edges += [b for b in bps if p.s0 + 1e-6 < b < p.s0 + p.length - 1e-6]
            edges = sorted(set(round(e, 9) for e in edges))

            for a, b in zip(edges, edges[1:]):
                if b - a < 1e-6:
                    continue
                yield from self._refine(a, b, p, rt, pt, min_step)

    def _refine(self, a, b, p, rt, pt, min_step):
        fa, fb = self.frame_at(a), self.frame_at(b)
        twisted = (abs(fa.roll - fb.roll) > rt or abs(fa.pitch - fb.pitch) > pt)
        if twisted and (b - a) > min_step:
            m = 0.5 * (a + b)
            yield from self._refine(a, m, p, rt, pt, min_step)
            yield from self._refine(m, b, p, rt, pt, min_step)
        else:
            yield (a, b, p)


def resolve_pads(layout, row_length: float):
    """Give every row the same length by sizing its `pad` segments.

    Sekil 1 draws three rows of equal length. Left to itself the layout will not
    produce that: the stages have fixed lengths set by the sartname, and they do
    not happen to add up the same way on each row. So each row declares filler
    segments (kind: pad) and this function shares the leftover between them.

    The result is that `row_length` is a single knob controlling the whole
    footprint, and the stages keep their required dimensions.

    Returns (resolved_layout, report) where report maps row -> (fixed, pad_each).
    """
    layout = [dict(item) for item in layout]
    report = {}

    rows = []
    for item in layout:
        r = item.get("row")
        if r is not None and r not in rows:
            rows.append(r)

    for row in rows:
        members = [it for it in layout if it.get("row") == row]
        pads = [it for it in members if it.get("kind") == "pad"]
        fixed = sum(float(it.get("length", 0.0))
                    for it in members if it.get("kind") != "pad")

        slack = row_length - fixed
        if not pads:
            if abs(slack) > 0.01:
                raise ValueError(
                    f"row {row!r} is {fixed:.2f} m of fixed segments but "
                    f"row_length is {row_length:.2f} m, and it has no pad "
                    f"segments to absorb the {slack:+.2f} m difference.")
            report[row] = (fixed, 0.0)
            continue

        if slack < -0.01:
            raise ValueError(
                f"row {row!r} holds {fixed:.2f} m of fixed segments, which "
                f"already exceeds row_length {row_length:.2f} m. Raise "
                f"track.row_length to at least {fixed:.1f}.")

        each = slack / len(pads)
        for p in pads:
            p["length"] = each
            p["kind"] = "straight"
        report[row] = (fixed, each)

    return layout, report


def build_from_layout(layout, turn_radius: float, origin=(0.0, 0.0),
                      heading_deg: float = 180.0,
                      profile: Profile | None = None) -> Centerline:
    """Build a Centerline from the `layout:` list in config.yaml."""
    prims = []
    for item in layout:
        kind = item.get("kind", "straight")
        common = dict(id=item["id"], label=item.get("label", ""),
                      stage=item.get("stage"))
        if kind == "straight":
            prims.append(Straight(length=float(item["length"]), **common))
        elif kind == "arc":
            prims.append(Arc(radius=float(item.get("radius", turn_radius)),
                             sweep_deg=float(item.get("sweep_deg", 180.0)),
                             turn=item["turn"], **common))
        else:
            raise ValueError(f"unknown layout kind {kind!r} in segment {item['id']!r}")
    return Centerline(prims, profile=profile, origin=origin, heading_deg=heading_deg)
