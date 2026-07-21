# models/

Generated assets. Do not hand-edit — `generator/signage.py` overwrites them.

```
ika_isaretler/
  model.config
  meshes/
    stage_01..10.{obj,mtl,png}   S6.2  numbered aşama tabelaları
    hedef.{obj,mtl,png}          S6.10 A3 atış hedefi
```

These are the **only textured objects in the whole parkur**. Everything else is
flat grey primitives, by design. These two are textured because the autonomous
run has to *read* them rather than merely avoid them.

## Regenerating

```bash
cd generator && python3 generate.py --textures
```

Without `--textures` existing files are reused, so a normal run costs nothing.
Requires Pillow (`pip install pillow`).

Raise `signs.texture_px` / `target.texture_px` in `config.yaml` to mint
higher-resolution copies for training your detector.

## Font

§6.2 specifies **Arial Black**. It is not redistributable and is absent from a
stock Ubuntu, so `signage.py` walks a preference list by weight and reports
which one it used. To get the real thing:

```bash
sudo apt install ttf-mscorefonts-installer
cd generator && python3 generate.py --textures
```

Fallback order: Arial Black → Lato Black → Liberation Sans Bold → DejaVu Sans Bold.

## Why OBJ + MTL instead of an SDF material override

The texture is referenced from the `.mtl`, not from `<material><pbr>` in the
world file. Mesh-embedded materials have worked in Gazebo for a decade; the SDF
override path is newer and degrades to a flat colour **silently** if anything
about it is off. A sign that renders as a plain grey square still looks like a
sign — you would only find out when your detector never fired.
