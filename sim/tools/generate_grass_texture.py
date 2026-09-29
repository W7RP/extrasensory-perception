#!/usr/bin/env python3
"""Generate the tileable grass texture for sim/models/grass_ground.

Offline asset tooling. Deterministic (seeded) and seamless, so the model can
repeat it every 25 m. Multi-scale noise in greens, with patches of dry grass and
bare earth, so the ground reads as ground at 2 m and at 40 m.
"""
import argparse

import numpy as np
from PIL import Image


def periodic_noise(rng, size, cells):
    grid = rng.random((cells, cells))
    coords = np.arange(size) * cells / size
    i0 = np.floor(coords).astype(int)
    f = coords - i0
    i1 = (i0 + 1) % cells
    f = f * f * (3 - 2 * f)
    top = grid[i0][:, i0] * (1 - f)[None, :] + grid[i0][:, i1] * f[None, :]
    bot = grid[i1][:, i0] * (1 - f)[None, :] + grid[i1][:, i1] * f[None, :]
    return top * (1 - f)[:, None] + bot * f[:, None]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--size", type=int, default=2048)
    ap.add_argument("--out", default="sim/models/grass_ground/materials/textures/grass.jpg")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    n = sum(periodic_noise(rng, a.size, c) * w for c, w in ((8, 0.35), (32, 0.3), (128, 0.2), (512, 0.15)))
    fine = periodic_noise(rng, a.size, 1024)
    patch = periodic_noise(rng, a.size, 5) * 0.7 + periodic_noise(rng, a.size, 13) * 0.3
    green = np.stack([0.20 + 0.12 * n, 0.42 + 0.20 * n, 0.14 + 0.08 * n], -1)
    dry = np.stack([0.50 + 0.10 * n, 0.46 + 0.10 * n, 0.24 + 0.06 * n], -1)
    earth = np.stack([0.40 + 0.08 * n, 0.31 + 0.06 * n, 0.22 + 0.05 * n], -1)
    w_dry = 0.6 * np.clip((patch - 0.6) * 4, 0, 1)[..., None]
    w_earth = 0.8 * np.clip((patch - 0.82) * 6, 0, 1)[..., None]
    img = green * (1 - w_dry) + dry * w_dry
    img = img * (1 - w_earth) + earth * w_earth
    img *= (0.85 + 0.3 * fine)[..., None]
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(a.out, quality=88)
    print(a.out)


if __name__ == "__main__":
    main()
