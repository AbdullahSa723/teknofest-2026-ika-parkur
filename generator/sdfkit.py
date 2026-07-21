"""
sdfkit - a very small SDF/XML construction helper.

The parkur is emitted as text rather than through an XML DOM library on
purpose: SDF files are read and hand-tweaked by humans, and a hand-rolled
pretty printer gives clean, diffable output with comments preserved.

Nothing here is Gazebo-specific beyond a handful of convenience builders at
the bottom of the file.
"""

from __future__ import annotations

INDENT = "  "


def num(v) -> str:
    """Format a number the way SDF likes it: no float noise, no sci notation."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        s = f"{v:.6f}".rstrip("0").rstrip(".")
        return s if s not in ("", "-0") else "0"
    return str(v)


def vec(*vals) -> str:
    """Space-separated vector, e.g. vec(1, 2, 3) -> '1 2 3'."""
    if len(vals) == 1 and isinstance(vals[0], (list, tuple)):
        vals = tuple(vals[0])
    return " ".join(num(v) for v in vals)


class E:
    """An XML element.

    Children may be other E instances, strings, numbers, None (dropped), or
    lists/tuples (flattened). A single scalar child renders inline.
    """

    __slots__ = ("tag", "attrs", "children")

    def __init__(self, tag: str, *children, **attrs):
        self.tag = tag
        self.attrs = {k: attrs[k] for k in attrs if attrs[k] is not None}
        self.children = []
        self._absorb(children)

    def _absorb(self, items):
        for c in items:
            if c is None:
                continue
            if isinstance(c, (list, tuple)):
                self._absorb(c)
            else:
                self.children.append(c)

    def add(self, *children) -> "E":
        self._absorb(children)
        return self

    # -- rendering ---------------------------------------------------------
    def _attr_str(self) -> str:
        if not self.attrs:
            return ""
        return "".join(f' {k}="{num(v)}"' for k, v in self.attrs.items())

    def render(self, level: int = 0) -> str:
        pad = INDENT * level
        head = f"{pad}<{self.tag}{self._attr_str()}"

        if not self.children:
            return head + "/>"

        # Single scalar child -> inline.
        if len(self.children) == 1 and not isinstance(self.children[0], E):
            return f"{head}>{num(self.children[0])}</{self.tag}>"

        lines = [head + ">"]
        for c in self.children:
            if isinstance(c, E):
                lines.append(c.render(level + 1))
            elif isinstance(c, Comment):
                lines.append(c.render(level + 1))
            else:
                lines.append(INDENT * (level + 1) + num(c))
        lines.append(f"{pad}</{self.tag}>")
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.render(0)


class Comment:
    """An XML comment. Used to keep sartname references inside the world."""

    __slots__ = ("text",)

    def __init__(self, text: str):
        # XML forbids '--' inside a comment. Collapse it rather than emitting
        # a file that xmllint (and sdformat) will reject.
        while "--" in text:
            text = text.replace("--", "-")
        self.text = text.rstrip("-")

    def render(self, level: int = 0) -> str:
        pad = INDENT * level
        if "\n" not in self.text:
            return f"{pad}<!-- {self.text} -->"
        body = "\n".join(f"{pad}     {ln}" for ln in self.text.split("\n"))
        return f"{pad}<!--\n{body}\n{pad}-->"


def banner(title: str, note: str = "") -> Comment:
    """A visually obvious section divider inside the generated SDF."""
    rule = "=" * 74
    text = f"{rule}\n {title}"
    if note:
        text += f"\n {note}"
    text += f"\n{rule}"
    return Comment(text)


# ---------------------------------------------------------------------------
#  Convenience builders
# ---------------------------------------------------------------------------

def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> E:
    return E("pose", vec(x, y, z, roll, pitch, yaw))


def material(rgba, specular=None) -> E:
    """Flat colour. rgba is a 4-tuple in 0..1."""
    spec = specular if specular is not None else (0.1, 0.1, 0.1, 1.0)
    return E(
        "material",
        E("ambient", vec(rgba)),
        E("diffuse", vec(rgba)),
        E("specular", vec(spec)),
    )


def friction(mu=1.0, mu2=1.0) -> E:
    return E(
        "surface",
        E("friction", E("ode", E("mu", mu), E("mu2", mu2))),
        E("contact", E("ode", E("kp", 1e6), E("kd", 1.0))),
    )


def box_link(name, size, rgba, link_pose=None, mu=1.0, collide=True) -> E:
    """A single-box link with matching visual and (optionally) collision."""
    geom = E("geometry", E("box", E("size", vec(size))))
    parts = [link_pose] if link_pose else []
    if collide:
        parts.append(E("collision", E("geometry", E("box", E("size", vec(size)))),
                       friction(mu, mu), name=f"{name}_collision"))
    parts.append(E("visual", geom, material(rgba), name=f"{name}_visual"))
    return E("link", *parts, name=name)


def static_box(name, size, position, rgba, yaw=0.0, roll=0.0, pitch=0.0,
               mu=1.0, collide=True) -> E:
    """A static, single-box model placed at `position` (its geometric centre)."""
    return E(
        "model",
        E("static", True),
        pose(position[0], position[1], position[2], roll, pitch, yaw),
        box_link(f"{name}_link", size, rgba, mu=mu, collide=collide),
        name=name,
    )


def sdf_document(world: E, version: str = "1.10") -> str:
    return (
        '<?xml version="1.0" ?>\n'
        f'<sdf version="{version}">\n'
        + world.render(1)
        + "\n</sdf>\n"
    )
