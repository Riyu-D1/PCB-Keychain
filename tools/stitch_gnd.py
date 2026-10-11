#!/usr/bin/env python3
"""Stitch isolated GND pour islands back into the net with vias.

Runs AFTER fill_zones.py: inspects the filled GND pours, finds islands
that contain GND pads but are cut off from the pour mainland, and drops
a GND via inside each at a spot also inside a connected region of the
opposite layer.  Then refills so the new vias get proper pour clearance.

Usage (KiCad-bundled python):
    .../python3.9 tools/stitch_gnd.py
"""
import sys
import os
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad", "keychain.kicad_pcb")

sys.path.insert(0, "/Applications/KiCad/KiCad.app/Contents/Frameworks/"
                 "Python.framework/Versions/3.9/lib/python3.9/site-packages")
import pcbnew  # noqa: E402

U = 1e6                    # internal units per mm
VIA_DRILL = int(0.25 * U)
VIA_DIA = int(0.5 * U)
CLEAR = int(0.15 * U)
GRID = int(0.25 * U)       # candidate spacing
KEEPOUTS = [(69.7, 43.0, 88.0, 59.0)]


def islands(zone, lid):
    """list of SHAPE_POLY_SET, one per filled island on layer `lid`."""
    if not zone.HasFilledPolysForLayer(lid):
        return []
    polys = zone.GetFilledPolysList(lid)
    out = []
    for i in range(polys.OutlineCount()):
        ps = pcbnew.SHAPE_POLY_SET()
        ps.AddOutline(polys.Outline(i))
        out.append(ps)
    return out


def bbox(ps):
    bb = ps.BBox()
    u = 1e6
    return (bb.GetX() / u, bb.GetY() / u,
            (bb.GetX() + bb.GetWidth()) / u,
            (bb.GetY() + bb.GetHeight()) / u)


def inside(ps, x_mm, y_mm):
    return ps.Collide(pcbnew.VECTOR2I(int(x_mm * U), int(y_mm * U)))


def main():
    board = pcbnew.LoadBoard(BOARD)
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())

    gnd = [z for z in board.Zones() if z.GetNetname() == "GND"]
    gnd_net = gnd[0].GetNetCode() if gnd else None
    if gnd_net is None:
        print("no GND zone")
        return
    # island outlines per layer, biggest first (mainland)
    isl = {}
    for z in gnd:
        lid = z.GetLayer()
        cur = islands(z, lid)
        isl.setdefault(lid, []).extend(cur)
    for lid in isl:
        isl[lid].sort(key=lambda ps: -ps.BBox().GetArea())
    f_isl, b_isl = isl.get(pcbnew.F_Cu, []), isl.get(pcbnew.B_Cu, [])
    print("F islands", len(f_isl), "B islands", len(b_isl))

    # obstacle model: every pad/via/track not on GND blocks via sites.
    # tracks are stored as segments (distance to the segment, not just
    # endpoints); pads/vias as circles.
    segs = []   # (x0,y0,x1,y1,radius_mm, foreign, layer)
    obst = []   # (x_mm, y_mm, radius_mm, foreign)
    for t in board.GetTracks():
        nc = t.GetNetCode()
        if t.Type() == pcbnew.PCB_VIA_T:
            # every via blocks new sites: hole-to-hole clearance applies
            # regardless of net.  store annulus + drill separately so
            # track and via clearance can each be derived.
            p = t.GetPosition()
            obst.append((p.x / U, p.y / U, t.GetWidth() / 2 / U,
                         t.GetDrillValue() / 2 / U, True))
        else:
            r = (t.GetWidth() / 2 + VIA_DIA / 2 + CLEAR) / U
            a, b = t.GetStart(), t.GetEnd()
            segs.append((a.x / U, a.y / U, b.x / U, b.y / U,
                         r, nc != gnd_net,
                         "F" if "F" in t.GetLayerName() else "B"))
    gndvias = [(t.GetPosition().x / U, t.GetPosition().y / U)
               for t in board.GetTracks()
               if t.Type() == pcbnew.PCB_VIA_T and
               t.GetNetCode() == gnd_net]
    gndpad = []  # (x,y) of GND pads (candidate via-in-pad sites)
    padrects = []  # (x0,y0,x1,y1, foreign)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            bb = p.GetBoundingBox()
            c = p.GetPosition()
            padrects.append((bb.GetX() / U, bb.GetY() / U,
                             (bb.GetX() + bb.GetWidth()) / U,
                             (bb.GetY() + bb.GetHeight()) / U,
                             p.GetNetCode() != gnd_net,
                             p.GetDrillSizeX() > 0))
            if p.GetNetCode() == gnd_net:
                gndpad.append((c.x / U, c.y / U))

    def rectdist(x, y, r):
        nx = min(max(x, r[0]), r[2])
        ny = min(max(y, r[1]), r[3])
        return ((x - nx) ** 2 + (y - ny) ** 2) ** 0.5

    def segdist(x, y, s):
        ax, ay, bx, by = s[0], s[1], s[2], s[3]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(
            0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        nx, ny = ax + t * dx, ay + t * dy
        return ((x - nx) ** 2 + (y - ny) ** 2) ** 0.5

    vneed = VIA_DIA / U / 2 + CLEAR / U
    VDR = VIA_DRILL / U / 2
    VDIA = VIA_DIA / U / 2

    def clear(x, y):
        for ox, oy, ar, dr, foreign in obst:
            need = max(dr + VDR + 0.25, ar + VDIA + CLEAR / U + 0.11)
            if foreign and (x - ox) ** 2 + (y - oy) ** 2 < need ** 2:
                return False
        for r in padrects:
            if r[4] and rectdist(x, y, r) < vneed:
                return False
        for s in segs:
            if s[5] and segdist(x, y, s) < s[4]:
                return False
        for kx0, ky0, kx1, ky1 in KEEPOUTS:
            if kx0 - 0.6 <= x <= kx1 + 0.6 and ky0 - 0.6 <= y <= ky1 + 0.6:
                return False
        return True

    def add_via(x, y):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(int(x * U), int(y * U)))
        v.SetDrill(VIA_DRILL)
        v.SetWidth(VIA_DIA)
        v.SetNetCode(gnd_net)
        board.Add(v)
        print(f"  GND via @({x:.2f},{y:.2f})")

    def add_track(x0, y0, x1, y1, layer_lid, width=0.25):
        tr = pcbnew.PCB_TRACK(board)
        tr.SetStart(pcbnew.VECTOR2I(int(x0 * U), int(y0 * U)))
        tr.SetEnd(pcbnew.VECTOR2I(int(x1 * U), int(y1 * U)))
        tr.SetWidth(int(width * U))
        tr.SetLayer(layer_lid)
        tr.SetNetCode(gnd_net)
        board.Add(tr)
        print(f"  GND track ({x0:.2f},{y0:.2f})->({x1:.2f},{y1:.2f})")

    import math as _m

    TRK_HALF = 0.4 / 2
    tneed = TRK_HALF + CLEAR / U

    def path_clear(x, y):
        """clear() for a 0.4mm track rather than a via."""
        for ox, oy, ar, dr, foreign in obst:
            need = max(dr + 0.25, ar) + TRK_HALF + CLEAR / U
            if foreign and (x - ox) ** 2 + (y - oy) ** 2 < need ** 2:
                return False
        for r in padrects:
            if r[4] and rectdist(x, y, r) < tneed:
                return False
        for s in segs:
            if s[5] and segdist(x, y, s) < s[4] - 0.1:
                return False
        for kx0, ky0, kx1, ky1 in KEEPOUTS:
            if kx0 - 0.3 <= x <= kx1 + 0.3 and ky0 - 0.3 <= y <= ky1 + 0.3:
                return False
        return True

    def track_stub(gx, gy, targets, layer_lid, same_mainland=None):
        """Last resort: run a GND track out of the island.  Two exits:
        the track may simply *end* inside the same-layer mainland pour
        (same-net copper touching pour is a connection), or reach a via
        site that lands inside a target region on the other layer."""
        ang = 0.0
        while ang < 6.2832:
            dx, dy = _m.cos(ang), _m.sin(ang)
            dist = 1.0
            while dist <= 15.0:
                vx, vy = gx + dx * dist, gy + dy * dist
                hit_main = (same_mainland is not None
                            and inside(same_mainland, vx, vy))
                hit_opp = (clear(vx, vy)
                           and any(inside(t, vx, vy) for t in targets))
                if hit_main or hit_opp:
                    n = int(dist / 0.2) + 1
                    ok = all(path_clear(gx + dx * dist * i / n,
                                        gy + dy * dist * i / n)
                             for i in range(1, n + 1))
                    if ok:
                        add_track(gx, gy, vx, vy, layer_lid)
                        if hit_opp and not hit_main:
                            add_via(vx, vy)
                        return True
                dist += 0.25
            ang += 0.1963  # 11.25 deg
        return False

    def free_l(x, y, layer, trk=0.25):
        """path_clear restricted to one layer: foreign copper only blocks
        a GND track on its own layer; SMD pads block F only, drilled
        pads and vias block both."""
        need = trk / 2 + CLEAR / U
        for ox, oy, ar, dr, foreign in obst:
            on = max(dr + 0.25, ar) + trk / 2 + CLEAR / U
            if foreign and (x - ox) ** 2 + (y - oy) ** 2 < on ** 2:
                return False
        for r in padrects:
            if r[4] and (layer == "F" or r[5]) \
                    and rectdist(x, y, r) < need:
                return False
        for s in segs:
            if s[5] and s[6] == layer and segdist(x, y, s) < s[4] - 0.1:
                return False
        for kx0, ky0, kx1, ky1 in KEEPOUTS:
            if kx0 - 0.3 <= x <= kx1 + 0.3 and ky0 - 0.3 <= y <= ky1 + 0.3:
                return False
        return True

    def escape_track(component):
        """BFS a GND track out of a dead island component.

        Straight-ray stubs miss bent escapes through wall joints; a
        same-layer grid BFS finds them — the dead zone's corridors wrap
        around wall ends that straight rays can't reach.  The track just
        needs to end inside the mainland pour of that layer: same-net
        copper touching pour is a connection."""
        for (layer, idx) in component:
            members = [ps for j, ps in
                       enumerate(f_isl if layer == "F" else b_isl)
                       if (layer, j) in component]
            mainland = (f_isl if layer == "F" else b_isl)[0]
            x0 = min(bbox(ps)[0] for ps in members) - 4
            y0 = min(bbox(ps)[1] for ps in members) - 4
            x1 = max(bbox(ps)[2] for ps in members) + 4
            y1 = max(bbox(ps)[3] for ps in members) + 4
            G = 0.25
            ix0, iy0 = int(x0 / G), int(y0 / G)
            nx, ny = int((x1 - x0) / G) + 1, int((y1 - y0) / G) + 1
            starts, goals = [], set()
            for ix in range(nx):
                for iy in range(ny):
                    cx, cy = (ix0 + ix) * G, (iy0 + iy) * G
                    if inside(mainland, cx, cy):
                        if free_l(cx, cy, layer):
                            goals.add((ix, iy))
                        continue
                    if not free_l(cx, cy, layer):
                        continue
                    if any(inside(ps, cx, cy) for ps in members):
                        starts.append((ix, iy))
            if not starts or not goals:
                continue
            from collections import deque
            prev = {s: None for s in starts}
            q = deque(starts)
            hit = None
            while q and hit is None:
                c = q.popleft()
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    n = (c[0] + dx, c[1] + dy)
                    if n in prev:
                        continue
                    if n in goals:
                        prev[n] = c
                        hit = n
                        break
                    cx, cy = (ix0 + n[0]) * G, (iy0 + n[1]) * G
                    if not (0 <= n[0] < nx and 0 <= n[1] < ny):
                        continue
                    if free_l(cx, cy, layer):
                        prev[n] = c
                        q.append(n)
            if hit is None:
                continue
            path = [hit]
            while prev[path[-1]] is not None:
                path.append(prev[path[-1]])
            path.reverse()
            # run-length emit: one segment per straight run
            lid = pcbnew.F_Cu if layer == "F" else pcbnew.B_Cu

            def cm(c):
                return ((ix0 + c[0]) * G, (iy0 + c[1]) * G)
            run0 = path[0]
            prev_dir = None
            for i in range(1, len(path)):
                nd = (path[i][0] - path[i - 1][0],
                      path[i][1] - path[i - 1][1])
                if prev_dir is not None and nd != prev_dir:
                    a, b = cm(run0), cm(path[i - 1])
                    if a != b:
                        add_track(a[0], a[1], b[0], b[1], lid)
                    run0 = path[i - 1]
                prev_dir = nd
            a, b = cm(run0), cm(path[-1])
            if a != b:
                add_track(a[0], a[1], b[0], b[1], lid)
            print(f"  GND escape track on {layer}: {len(path)} cells")
            return True
        return False

    def touches(ps, x, y):
        """via annulus reaches island copper: centre or a ring point on
        the via edge lands inside the island poly."""
        if inside(ps, x, y):
            return True
        # island copper must reach ~0.32 past the via edge to survive the
        # pour's 0.3 clearance pull-back around the via
        r = VIA_DIA / U / 2 + 0.32
        for a in range(12):
            th = a / 12 * 6.2831853
            if inside(ps, x + r * __import__("math").cos(th),
                        y + r * __import__("math").sin(th)):
                return True
        return False

    def stitch(islands_list, targets, layer_lid):
        """every non-mainland island needs a via into a target region."""
        placed = 0
        for ps in islands_list[1:]:
            # already stitched by a GND via inside (or overlapping) it?
            if any(touches(ps, vx, vy) for vx, vy in gndvias):
                continue
            x0, y0, x1, y1 = bbox(ps)
            done = False
            yy = y0 - 0.7
            while yy <= y1 + 0.7 and not done:
                xx = x0 - 0.7
                while xx <= x1 + 0.7 and not done:
                    if touches(ps, xx, yy) and clear(xx, yy):
                        for tgt in targets:
                            if inside(tgt, xx, yy):
                                add_via(xx, yy)
                                done = True
                                placed += 1
                                break
                    xx += 0.1
                yy += 0.1
            if not done:
                # last resort: via-in-pad near a GND pad inside the island
                # (electrically fine, mechanically fine for a keychain).
                # try the pad centre plus a small ring around it.
                cands = []
                for gx, gy in gndpad:
                    if touches(ps, gx, gy):
                        cands.append((gx, gy))
                        for dx in (-0.6,-0.3, 0.0, 0.3,0.6):
                            for dy in (-0.6,-0.3, 0.0, 0.3,0.6):
                                cands.append((gx + dx, gy + dy))
                for gx, gy in cands:
                    if done:
                        break
                    if not (touches(ps, gx, gy) and clear(gx, gy)):
                        continue
                    for tgt in targets:
                        if inside(tgt, gx, gy):
                            add_via(gx, gy)
                            done = True
                            placed += 1
                            break
            if not done:
                # last resort 2: run a GND track out of the island to a
                # via site on a connected region of the other layer.
                for gx, gy in gndpad:
                    if done:
                        break
                    if touches(ps, gx, gy) or inside(ps, gx, gy):
                        if track_stub(gx, gy, targets, layer_lid):
                            done = True
                            placed += 1
            if not done:
                print(f"  ! no via site in island {bbox(ps)}")
        return placed

    def via_bridges():
        """edges: which (layer,island) pairs each GND via joins."""
        edges = []
        for vx, vy in gndvias:
            fn = [i for i, ps in enumerate(f_isl) if inside(ps, vx, vy)]
            bn = [i for i, ps in enumerate(b_isl) if inside(ps, vx, vy)]
            for i in fn:
                for j in bn:
                    edges.append((("F", i), ("B", j)))
        # GND tracks bridge same-layer islands the pour itself can't
        for t in board.GetTracks():
            if t.GetNetname() != "GND" or t.Type() == pcbnew.PCB_VIA_T:
                continue
            a, e = t.GetStart(), t.GetEnd()
            L = "F" if "F" in t.GetLayerName() else "B"
            lst = f_isl if L == "F" else b_isl
            ids = set()
            for i, ps in enumerate(lst):
                for tt in (0, .25, .5, .75, 1):
                    x = (a.x + (e.x - a.x) * tt) / U
                    y = (a.y + (e.y - a.y) * tt) / U
                    if inside(ps, x, y):
                        ids.add(i)
            ids = list(ids)
            for k in ids[1:]:
                edges.append(((L, ids[0]), (L, k)))
        return edges

    def components(edges):
        """union-find over (layer, island-index) nodes."""
        nodes = ([("F", i) for i in range(len(f_isl))] +
                 [("B", i) for i in range(len(b_isl))])
        par = {n: n for n in nodes}

        def find(a):
            while par[a] != a:
                par[a] = par[par[a]]
                a = par[a]
            return a
        for a, b in edges:
            par[find(a)] = find(b)
        out = {}
        for n in nodes:
            out.setdefault(find(n), set()).add(n)
        return out

    # stitch every island chain that lacks the mainland: place a via
    # from a member island into the mainland of the opposite layer,
    # then re-derive components.  bounded to a few passes — each pass
    # either merges a component or reports it unfixable.
    placed = 0
    for _pass in range(6):
        comp = components(via_bridges())
        main = next((c for c in comp.values()
                     if ("F", 0) in c or ("B", 0) in c), set())
        stuck = [c for c in comp.values() if c is not main]
        if not stuck:
            break
        progressed = False
        for c in stuck:
            done = False
            for (layer, idx) in c:
                if done:
                    break
                ps = (f_isl if layer == "F" else b_isl)[idx]
                targets = b_isl if layer == "F" else f_isl
                x0, y0, x1, y1 = bbox(ps)
                yy = y0 - 0.7
                while yy <= y1 + 0.7 and not done:
                    xx = x0 - 0.7
                    while xx <= x1 + 0.7 and not done:
                        # mainland only — landing on another island just
                        # grows the dead chain
                        if (touches(ps, xx, yy) and clear(xx, yy)
                                and inside(targets[0], xx, yy)):
                            add_via(xx, yy)
                            gndvias.append((xx, yy))
                            done = True
                            placed += 1
                            progressed = True
                        xx += 0.1
                    yy += 0.1
            if not done and escape_track(c):
                done = True
                placed += 1
                progressed = True
            if not done:
                print(f"  ! component stuck: {sorted(c)[:4]}")
        if not progressed:
            break
    print("placed", placed, "stitch vias")
    if placed:
        filler.Fill(board.Zones())
    board.Save(BOARD)


if __name__ == "__main__":
    main()
