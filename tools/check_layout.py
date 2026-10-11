#!/usr/bin/env python3
"""Fast placement sanity checker: courtyard + pad overlap report.

Pure-python (no pcbnew): parses each footprint's F.CrtYd primitives,
transforms to board coords via PLACEMENT, reports pairwise overlaps.
Pads are unioned into the obstacle rect too (courtyards in some libs
don't cover pad overhang).  Usage: python3 tools/check_layout.py
"""

import math
import re
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import generate as G


def courtyard_rect(qualname):
    """local-axis-aligned rect union of F.CrtYd + pads."""
    # U1's courtyard is shrunk to the module body in build_pcb —
    # mirror that here so overlap checks match what DRC will see
    if "ESP32-C6-MINI-1" in qualname:
        return -6.85, -11.25, 6.85, 5.85
    data = G.load_fp(qualname)
    xs, ys = [], []
    for m in re.finditer(r'\((fp_line|fp_rect|fp_poly|fp_circle|fp_arc|fp_pad|pad)\b', data):
        blk = G._block(data, m.start())
        if '(layer "F.CrtYd")' not in blk and m.group(1) != 'pad':
            continue
        for sm in re.finditer(
                r'\((?:start|end|xy|center|mid) ([-\d.]+) ([-\d.]+)\)', blk):
            xs.append(float(sm.group(1)))
            ys.append(float(sm.group(2)))
        at = re.search(r'\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', blk)
        sz = re.search(r'\(size ([-\d.]+) ([-\d.]+)\)', blk)
        if at and sz:
            ax, ay = float(at.group(1)), float(at.group(2))
            w, h = float(sz.group(1)) / 2, float(sz.group(2)) / 2
            rot = float(at.group(3) or 0)
            if rot % 180:
                w, h = h, w
            xs += [ax - w, ax + w]
            ys += [ay - h, ay + h]
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def board_rect(ref):
    fp = next(c[3] for c in G.COMPONENTS if c[0] == ref)
    x, y, rot, side = G.PLACEMENT[ref]
    x0, y0, x1, y1 = courtyard_rect(fp)
    # four corners through the transform
    pts = [G.xform(x, y, rot, side, px, py)
           for px in (x0, x1) for py in (y0, y1)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def main():
    rects = {}
    for c in G.COMPONENTS:
        ref = c[0]
        if ref.startswith("H"):   # mounting holes excluded
            continue
        rects[ref] = board_rect(ref)
    bad = []
    refs = sorted(rects)
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            ax0, ay0, ax1, ay1 = rects[a]
            bx0, by0, bx1, by1 = rects[b]
            if ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1:
                ox = min(ax1, bx1) - max(ax0, bx0)
                oy = min(ay1, by1) - max(ay0, by0)
                bad.append((a, b, round(ox, 2), round(oy, 2)))
    for a, b, ox, oy in bad:
        print(f"OVERLAP {a:5s} {b:5s} dx={ox} dy={oy}   "
              f"{a}:{rects[a]}  {b}:{rects[b]}")
    if not bad:
        print("no courtyard overlaps")
    # keepout sanity: no pad inside a keepout that forbids copper
    import router as R
    r = R.Router()
    for (kx0, ky0, kx1, ky1) in G.KEEPOUTS:
        for (ref, name), shapes in r.shapes.items():
            for ax, ay, w, h, dr in shapes:
                if (kx0 < ax + w / 2 and ax - w / 2 < kx1 and
                        ky0 < ay + h / 2 and ay - h / 2 < ky1):
                    print(f"PAD-IN-KEEPOUT {ref}.{name} "
                          f"({ax},{ay},{w}x{h}) ko=({kx0},{ky0},{kx1},{ky1})")


if __name__ == "__main__":
    main()
