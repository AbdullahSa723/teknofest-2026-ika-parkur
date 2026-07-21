"""
Texture and mesh generation for the two things in the parkur that have to be
READ rather than just collided with:

  * S6.2  numbered stage tabelalar, 60 cm outer diameter, 'Arial Black'
  * S6.10 the A3 atis target from Sekil 5, 6 / 12 / 18 cm rings

Everything else in the world is flat grey. These are textured because the
autonomous run depends on recognising them.

Each asset is emitted as a self-contained OBJ + MTL + PNG triple. The material
lives in the MTL rather than in an SDF <material><pbr> override: mesh-embedded
materials have worked in Gazebo forever, whereas the SDF override path is newer
and fails silently to a flat colour if anything about it is wrong. Silent
failure is the worst outcome here, because a sign that renders as a plain grey
square still looks like a sign until your detector quietly never fires.
"""

from __future__ import annotations

import math
import pathlib

try:
    from PIL import Image, ImageDraw, ImageFont
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False


# S6.2 asks for Arial Black. It is not redistributable and is absent from a
# stock Ubuntu, so this is a preference order by weight, heaviest first. Install
# ttf-mscorefonts-installer to get the real thing.
FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/msttcorefonts/Arial_Black.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/ariblk.ttf",
    "/usr/local/share/fonts/Arial_Black.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Black.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]

RED = (206, 32, 38)
BLACK = (16, 16, 16)
WHITE = (250, 250, 248)


def find_font():
    """(path, is_real_arial_black). Raises if nothing usable exists."""
    for p in FONT_CANDIDATES:
        if pathlib.Path(p).is_file():
            return p, "arial" in p.lower() or "ariblk" in p.lower()
    raise RuntimeError(
        "no heavy sans font found. Install one of:\n"
        "  sudo apt install fonts-lato            (closest weight)\n"
        "  sudo apt install ttf-mscorefonts-installer   (real Arial Black)")


def _fit_font(draw, text, font_path, target_w, target_h):
    """Largest font size whose rendered text fits inside the given box."""
    lo, hi = 8, int(target_h * 2.2)
    best = ImageFont.truetype(font_path, lo)
    while lo <= hi:
        mid = (lo + hi) // 2
        f = ImageFont.truetype(font_path, mid)
        l, t, r, b = draw.textbbox((0, 0), text, font=f)
        if (r - l) <= target_w and (b - t) <= target_h:
            best, lo = f, mid + 1
        else:
            hi = mid - 1
    return best


def stage_sign(number: int, px: int, cfg, cancelled: bool = False) -> "Image.Image":
    """One numbered tabela. S6.2: 60 cm outer diameter, 'Arial Black'.

    The image is FULL-BLEED RED rather than a red ring on white. The mesh it
    lands on is a disc whose rim sits exactly on the edge of the image, so a
    ring drawn to the same diameter would sample its own antialiased boundary
    and fringe the sign white. Flooding the background means the rim can only
    ever be red; the corners the flood fills are outside the disc and never
    rendered.

    `cancelled` strikes the number through with a red diagonal. Sekil 1 marks
    both ends of the hizlanma parkuru with the SAME number and cancels the one
    at the finish, the way an end-of-restriction sign cancels the sign that
    opened it. So the pair reads "11" and "11 struck through" rather than 11
    and 12, and the slash is the only thing the detector has to tell apart.
    """
    s = cfg["signs"]
    ring_m = s["outer_diameter"]
    scale = px / ring_m                        # pixels per metre

    img = Image.new("RGB", (px, px), RED)
    d = ImageDraw.Draw(img)
    c = px / 2.0

    r_in = (ring_m / 2.0 - s["ring_width"]) * scale
    d.ellipse([c - r_in, c - r_in, c + r_in, c + r_in], fill=WHITE)

    text = str(number)
    font_path, _ = find_font()
    font = _fit_font(d, text, font_path, 1.62 * r_in, 1.15 * r_in)
    l, t, r, b = d.textbbox((0, 0), text, font=font)
    d.text((c - (r + l) / 2.0, c - (b + t) / 2.0), text, font=font, fill=BLACK)

    if cancelled:
        # Lower-left to upper-right, the direction every end-of-restriction
        # sign uses. Drawn last so it sits ON the digits, and taken all the way
        # out to the ring so the plate cannot be mistaken for a plain number at
        # the distance a camera first picks it up. PIL's y grows downward while
        # the disc's UVs put v=1 at the top, so "up" here is a SMALLER y.
        arm = r_in * 0.99 * math.sqrt(0.5)
        d.line([(c - arm, c + arm), (c + arm, c - arm)],
               fill=RED, width=max(2, int(round(s["ring_width"] * scale))))
    return img


def stop_sign(text: str, px: int, cfg) -> "Image.Image":
    """A round DUR / STOP plate for the S6.10 stop line.

    Inverted against the numbered tabelalar on purpose: white lettering on a
    red field, where a stage sign is a black number on a white field inside a
    red ring. A stop plate that looked like a stage plate would be a detection
    hazard, not an aid -- the vehicle has to react to one and merely count the
    other.

    Round rather than the octagon a real DUR sign uses: every plate in this
    world is a disc mesh, and an octagon painted onto a round plate would read
    worse than an honest circle. Full-bleed red for the same rim-fringing
    reason as stage_sign.
    """
    s = cfg["signs"]
    scale = px / s["outer_diameter"]
    img = Image.new("RGB", (px, px), RED)
    d = ImageDraw.Draw(img)
    c = px / 2.0

    # White keyline inset from the rim, as on the real sign.
    r_out = (s["outer_diameter"] / 2.0 - 0.4 * s["ring_width"]) * scale
    r_mid = r_out - 0.35 * s["ring_width"] * scale
    d.ellipse([c - r_out, c - r_out, c + r_out, c + r_out], fill=WHITE)
    d.ellipse([c - r_mid, c - r_mid, c + r_mid, c + r_mid], fill=RED)

    font_path, _ = find_font()
    font = _fit_font(d, text, font_path, 1.55 * r_mid, 0.62 * r_mid)
    l, t, r, b = d.textbbox((0, 0), text, font=font)
    d.text((c - (r + l) / 2.0, c - (b + t) / 2.0), text, font=font, fill=WHITE)
    return img


def atis_target(px: int, cfg) -> "Image.Image":
    """Sekil 5: concentric 6 / 12 / 18 cm circles on A3.

    Reading the figure outward: a black disc in the middle worth 50, a white
    ring worth 25, a black ring worth 15.
    """
    t = cfg["target"]
    w_m, h_m = t["paper_w"], t["paper_h"]
    scale = px / w_m
    img = Image.new("RGB", (px, int(round(h_m * scale))), WHITE)
    d = ImageDraw.Draw(img)
    cx, cy = img.width / 2.0, img.height / 2.0

    def disc(diam_m, fill):
        r = diam_m / 2.0 * scale
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill)

    disc(t["ring_outer_d"], BLACK)      # 18 cm, dis halka   15 puan
    disc(t["ring_mid_d"], WHITE)        # 12 cm, orta halka  25 puan
    disc(t["ring_inner_d"], BLACK)      # 6 cm,  ic daire    50 puan

    # Hairline border so the sheet edge is visible against a pale background.
    d.rectangle([0, 0, img.width - 1, img.height - 1], outline=(180, 180, 180))
    return img


def write_quad(path_obj: pathlib.Path, name: str, width: float, height: float,
               texture: str) -> None:
    """A textured quad standing upright in the XZ plane, normal along -Y.

    Kept vertical in the mesh itself so placing it needs only a yaw. Building it
    flat and tipping it up with a pitch would also roll the image sideways.
    """
    hw, hh = width / 2.0, height / 2.0
    path_obj.write_text(
        f"# {name} - generated by generator/signage.py\n"
        f"mtllib {path_obj.stem}.mtl\n"
        f"o {name}\n"
        f"v {-hw:.6f} 0.000000 {-hh:.6f}\n"
        f"v {hw:.6f} 0.000000 {-hh:.6f}\n"
        f"v {hw:.6f} 0.000000 {hh:.6f}\n"
        f"v {-hw:.6f} 0.000000 {hh:.6f}\n"
        "vt 0.000000 0.000000\n"
        "vt 1.000000 0.000000\n"
        "vt 1.000000 1.000000\n"
        "vt 0.000000 1.000000\n"
        "vn 0.000000 -1.000000 0.000000\n"
        f"usemtl {name}_mat\n"
        "f 1/1/1 2/2/1 3/3/1 4/4/1\n",
        encoding="utf-8")

    path_obj.with_suffix(".mtl").write_text(
        f"newmtl {name}_mat\n"
        "Ka 1.000 1.000 1.000\n"
        "Kd 1.000 1.000 1.000\n"
        "Ks 0.000 0.000 0.000\n"
        "d 1.0\n"
        "illum 1\n"
        f"map_Kd {texture}\n",
        encoding="utf-8")


def write_disc(path_obj: pathlib.Path, name: str, diameter: float,
               texture: str, segments: int = 96) -> None:
    """A textured DISC standing upright in the XZ plane, normal along -Y.

    S6.2 says "dis capi 60 cm olan tabelalar" -- an outer DIAMETER, so the sign
    is round. Building the plate as real circular geometry keeps it round
    without an alpha channel, which is the failure-prone way to punch a circle
    out of a quad.

    UVs map the disc onto the inscribed circle of a square texture, so the
    image's own circle lands exactly on the rim.
    """
    r = diameter / 2.0
    verts = ["v 0.000000 0.000000 0.000000"]
    uvs = ["vt 0.500000 0.500000"]
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        verts.append(f"v {r * math.cos(a):.6f} 0.000000 {r * math.sin(a):.6f}")
        uvs.append(f"vt {0.5 + 0.5 * math.cos(a):.6f} {0.5 + 0.5 * math.sin(a):.6f}")

    faces = []
    for i in range(segments):
        a = i + 2
        b = (i + 1) % segments + 2
        faces.append(f"f 1/1/1 {a}/{a}/1 {b}/{b}/1")

    path_obj.write_text(
        f"# {name} - generated by generator/signage.py\n"
        f"mtllib {path_obj.stem}.mtl\n"
        f"o {name}\n"
        + "\n".join(verts) + "\n"
        + "\n".join(uvs) + "\n"
        "vn 0.000000 -1.000000 0.000000\n"
        f"usemtl {name}_mat\n"
        + "\n".join(faces) + "\n",
        encoding="utf-8")

    path_obj.with_suffix(".mtl").write_text(
        f"newmtl {name}_mat\n"
        "Ka 1.000 1.000 1.000\n"
        "Kd 1.000 1.000 1.000\n"
        "Ks 0.000 0.000 0.000\n"
        "d 1.0\n"
        "illum 1\n"
        f"map_Kd {texture}\n",
        encoding="utf-8")


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>ika_isaretler</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    Generated stage tabelalar (S6.2) and the atis target (S6.10) for the
    TEKNOFEST 2026 IKA parkuru. Regenerate with generator/generate.py.
  </description>
</model>
"""


def generate(cfg, models_dir: pathlib.Path, force: bool = False) -> dict:
    """Write every sign and target asset. Returns a small report."""
    pkg = models_dir / "ika_isaretler"
    meshes = pkg / "meshes"

    report = {"dir": pkg, "font": None, "arial": False,
              "written": 0, "skipped": 0}

    n_stages = cfg["signs"]["count"]
    drag_no = cfg["accel_strip"]["sign_start"]
    stops = (("cikis", cfg["ramp"]["stop_sign_climb"]),
             ("inis", cfg["ramp"]["stop_sign_descent"]))
    wanted = [meshes / f"stage_{i:02d}.png" for i in range(1, n_stages + 1)]
    wanted.append(meshes / f"stage_{drag_no:02d}_bitis.png")
    wanted += [meshes / f"dur_{tag}.png" for tag, _txt in stops]
    wanted.append(meshes / "hedef.png")
    if not force and all(p.is_file() for p in wanted):
        report["skipped"] = len(wanted)
        return report

    if not HAVE_PIL:
        raise RuntimeError(
            "Pillow is needed to generate the sign and target textures:\n"
            "  pip install pillow      (or: sudo apt install python3-pil)\n"
            "The numbered tabelalar of S6.2 cannot be built without it.")

    meshes.mkdir(parents=True, exist_ok=True)
    (pkg / "model.config").write_text(MODEL_CONFIG, encoding="utf-8")

    font_path, is_arial = find_font()
    report["font"], report["arial"] = font_path, is_arial

    px = cfg["signs"]["texture_px"]
    plate = cfg["signs"]["outer_diameter"]
    for i in range(1, n_stages + 1):
        stage_sign(i, px, cfg).save(meshes / f"stage_{i:02d}.png")
        write_disc(meshes / f"stage_{i:02d}.obj", f"stage_{i:02d}",
                   plate, f"stage_{i:02d}.png")
        report["written"] += 1

    # The cancelled twin of the drag strip's number, for its finish line.
    name = f"stage_{drag_no:02d}_bitis"
    stage_sign(drag_no, px, cfg, cancelled=True).save(meshes / f"{name}.png")
    write_disc(meshes / f"{name}.obj", name, plate, f"{name}.png")
    report["written"] += 1

    # S6.10 stop plates. Two wordings, one per slope -- see config.yaml.
    for tag, txt in stops:
        stop_sign(txt, px, cfg).save(meshes / f"dur_{tag}.png")
        write_disc(meshes / f"dur_{tag}.obj", f"dur_{tag}", plate,
                   f"dur_{tag}.png")
        report["written"] += 1

    t = cfg["target"]
    atis_target(cfg["target"]["texture_px"], cfg).save(meshes / "hedef.png")
    write_quad(meshes / "hedef.obj", "hedef", t["paper_w"], t["paper_h"],
               "hedef.png")
    report["written"] += 1
    return report
