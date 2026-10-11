#!/usr/bin/env python3
"""Courtyard-overlap checker for the generated keychain.kicad_pcb.

Pure python: parses each placed footprint's F.CrtYd primitives + pads,
transforms to board coords via PLACEMENT in place_board.py, and reports
pairwise AABB overlaps plus out-of-bounds parts vs the board rect.
"""

import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate as G
import place_board as P


def local_rect(qualname):
    data = G.load_fp(qualname)
    xs, ys = [], []
    for m in re.finditer(
            r'\((fp_line|fp_rect|fp_poly|fp_circle|fp_arc|pad)\b', data):
        blk = G._block(data, m.start())
        is_cy = 'CrtYd' in blk
        is_pad = m.group(1) == 'pad'
        if not (is_cy or is_pad):
            continue
        for sm in re.finditer(
                r'\((?:start|end|xy|center|mid) "?([-\d.]+)"? "?([-\d.]+)"?\)',
                blk):
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
        dr = re.search(r'\(drill (?:\S+ )?([-\d.]+)\)', blk)
        if at and dr:
            r = float(dr.group(1)) / 2
            ax, ay = float(at.group(1)), float(at.group(2))
            xs += [ax - r, ax + r]
            ys += [ay - r, ay + r]
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def board_rect(ref, fp):
    x, y, rot, side = P.PLACEMENT[ref]
    r = local_rect(fp)
    if r is None:
        return None
    x0, y0, x1, y1 = r
    pts = [G.xform(x, y, rot, side, px, py)
           for px in (x0, x1) for py in (y0, y1)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def main():
    comps, _nets = P.parse_netlist(P.NETLIST)
    fpnames = dict(comps)  # ref -> (fp, val, lp)
    fpnames.update({k: (v[0], v[1], "") for k, v in P.EXTRA_FOOTPRINTS.items()})
    rects = {}
    for ref, (fp, *_a) in fpnames.items():
        r = board_rect(ref, fp)
        if r:
            rects[ref] = r
        else:
            print(f"{ref}: no courtyard found!")
    refs = sorted(rects)
    nbad = 0
    for i, a in enumerate(refs):
        ax0, ay0, ax1, ay1 = rects[a]
        # out of bounds check (allow intentional edge protrusions: J1/SW1/J2 lips)
        protrude = {
            "J1": (None, None, None, P.BY1 + 0.6),
            "SW1": (P.BX0 - 0.5, None, None, None),
            "J2": (P.BX0 - 0.5, None, None, None),
            "J3": (None, None, None, P.BY1 + 0.1),
        }.get(a, (P.BX0, P.BY0, P.BX1, P.BY1))
        lx, ty, rx, by = protrude
        oob = []
        if lx is not None and ax0 < lx: oob.append('L')
        if ty is not None and ay0 < ty: oob.append('T')
        if rx is not None and ax1 > rx: oob.append('R')
        if by is not None and ay1 > by: oob.append('B')
        if oob:
            print(f"OOB {a}: rect {ax0:.2f},{ay0:.2f}-{ax1:.2f},{ay1:.2f} "
                  f"past {','.join(oob)}")
            nbad += 1
        for b in refs[i + 1:]:
            if P.PLACEMENT[a][3] != P.PLACEMENT[b][3]:
                continue
            bx0, by0, bx1, by1 = rects[b]
            ox = min(ax1, bx1) - max(ax0, bx0)
            oy = min(ay1, by1) - max(ay0, by0)
            if ox > 0.02 and oy > 0.02:
                print(f"OVERLAP {a} {b}: {ox:.2f} x {oy:.2f} mm")
                nbad += 1
    print(f"{len(rects)} footprints checked, {nbad} problems")


if __name__ == "__main__":
    main()
