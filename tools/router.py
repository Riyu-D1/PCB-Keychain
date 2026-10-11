#!/usr/bin/env python3
"""Grid-based A* router for the keychain board.

Deterministic, bounded (MAX_* caps everywhere) two-layer Manhattan router.
Works on generator data: pad geometry comes from the kicad_mod footprints
via generate.fp_pads plus PLACEMENT transforms — no pcbnew dependency.

Usage (from generate.py):
    import router
    tracks, failed = router.route_all()   # -> segment/via records
"""

import heapq
import math
import re

import generate as G

# ---------------------------------------------------------------------------
# design rules / router tuning
# ---------------------------------------------------------------------------
GRID = 0.25                 # mm, route pitch
CLEAR = 0.15                # min copper clearance
EDGE_CLEAR = 0.3            # copper to board edge
VIA_DRILL = 0.4
VIA_DIA = 0.8
VIA_COST = 4.0              # mm-equivalent penalty per layer change
MAX_ASTAR_EXPANSIONS = 250_000   # per pad-pair search; hard stop
MAX_NET_ATTEMPTS = 4             # retry budget per net

POWER_W = 0.45
SIGNAL_W = 0.25
WIDTHS = {"VBUS": POWER_W, "VBUS_F": POWER_W, "VBAT": POWER_W,
          "VBAT_SYS": POWER_W, "+3V3": 0.4, "+5V": POWER_W,
          "BOOST_SW": 0.35, "USB_DP": 0.3, "USB_DM": 0.3}

# route order: short local nets first, then the power rails while
# corridors are still open, then long signal runs last
ORDER = [
    # the fine-pitch USB-C escape is the most constrained: route it first,
    # middle pads before outer ones so early tracks cannot wall in the
    # interior fan-out lanes
    "VBUS", "VBUS_F",
    "USB_DM_CONN", "USB_DP_CONN", "CC1", "CC2",
    # interior pads near the MCU/driver are the next most enclosure-prone:
    # route them before long runs claim the corridors around them
    "LED_DAT_3V", "LED_DIN",
    "OLED_SCK", "OLED_MOSI", "OLED_RES", "OLED_DC", "OLED_CS",
    "USER1", "USER2", "BOOT", "EN", "CHG_STAT",
    "USB_DP", "USB_DM",
    "CHG_LED", "PROG", "PWR_LED", "BAT_ADC",
    "+3V3", "+5V", "VBAT_SYS", "VBAT",
    "BOOST_SW", "LED_FB", "BOOST_EN",
    "LED_D0", "LED_C1", "LED_C2", "LED_C3",
]


# ---------------------------------------------------------------------------
# pad / obstacle geometry
# ---------------------------------------------------------------------------

def pads_of(qualname):
    """padname -> list of (x, y, w, h, rot, drill|None)."""
    data = G.load_fp(qualname)
    out = {}
    for m in re.finditer(r'\(pad "([^"]*)"', data):
        pb = G._block(data, m.start())
        at = re.search(r'\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', pb)
        sz = re.search(r'\(size ([-\d.]+) ([-\d.]+)\)', pb)
        dr = re.search(r'\(drill(?: oval)? ([-\d.]+)(?: ([-\d.]+))?\)', pb)
        if not (at and sz):
            continue
        out.setdefault(m.group(1), []).append(
            (float(at.group(1)), float(at.group(2)),
             float(sz.group(1)), float(sz.group(2)),
             float(at.group(3) or 0.0),
             (float(dr.group(1)), float(dr.group(2) or dr.group(1)))
             if dr else None))
    return out


_FPINFO = {}


def fpinfo(ref):
    if not _FPINFO:
        for r, libsym, value, fp, *_ in G.COMPONENTS:
            _FPINFO[r] = (fp, pads_of(fp))
    fp, padmap = _FPINFO[ref]
    x, y, rot, side = G.PLACEMENT[ref]
    return x, y, rot, side, fp, padmap


def pad_shapes_abs(ref, name):
    """absolute rects for every physical pad with this number."""
    x, y, rot, side, fp, padmap = fpinfo(ref)
    out = []
    for px, py, w, h, prot, drill in padmap[name]:
        br = (prot + rot) % 360 if side == "F" else (rot - prot) % 360
        ax, ay = G.xform(x, y, rot, side, px, py)
        if br % 180:
            w, h = h, w
        out.append((ax, ay, w, h, drill))
    return out


class Rect:
    __slots__ = ("x0", "y0", "x1", "y1")

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1

    def inflate(self, d):
        return Rect(self.x0 - d, self.y0 - d, self.x1 + d, self.y1 + d)

    def inside(self, x, y):
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


class Router:
    def __init__(self):
        # (ref,pad) -> list of absolute shapes, and -> net
        self.shapes = {}
        self.netof = {}
        pinnet = {}
        for net, pins in G.NETS.items():
            for rp in pins:
                pinnet[rp] = net
        for ref, libsym, value, fp, *_ in G.COMPONENTS:
            _, _, _, _, _, padmap = fpinfo(ref)
            for name in padmap:
                self.shapes[(ref, name)] = pad_shapes_abs(ref, name)
                self.netof[(ref, name)] = pinnet.get((ref, name))

        self.pads_by_net = {}
        for rp, net in self.netof.items():
            if net:
                self.pads_by_net.setdefault(net, []).append(rp)

        # keep-in regions (board): main rect + lug
        self.keepin = [Rect(20, 25, 88, 77), Rect(26, 13, 42, 25)]
        self.keepouts = [Rect(*k) for k in G.KEEPOUTS]
        # committed copper (for obstacle marking between nets)
        self.segs = []   # (layer, x0,y0,x1,y1, width, net)
        self.vias = []   # (x, y, net)

    def _cells(self, rect):
        g = GRID
        for ix in range(int(math.floor(rect.x0 / g)),
                        int(math.floor(rect.x1 / g)) + 1):
            for iy in range(int(math.floor(rect.y0 / g)),
                            int(math.floor(rect.y1 / g)) + 1):
                yield ix, iy

    def _block_rect(self, occ, layer, rect):
        """block cells whose CENTER falls inside rect (centerline model)."""
        g = GRID
        for ix in range(int(math.floor(rect.x0 / g)),
                        int(math.floor(rect.x1 / g)) + 1):
            cx = ix * g
            if not rect.x0 <= cx <= rect.x1:
                continue
            for iy in range(int(math.floor(rect.y0 / g)),
                            int(math.floor(rect.y1 / g)) + 1):
                cy = iy * g
                if rect.y0 <= cy <= rect.y1:
                    occ[layer][(ix, iy)] = True

    def _block_seg(self, occ, layer, ax, ay, bx, by, half, collect=None):
        """block cell centers closer than `half` to segment a->b."""
        g = GRID
        r = Rect(min(ax, bx) - half, min(ay, by) - half,
                 max(ax, bx) + half, max(ay, by) + half)
        for ix in range(int(math.floor(r.x0 / g)),
                        int(math.floor(r.x1 / g)) + 1):
            cx = ix * g
            for iy in range(int(math.floor(r.y0 / g)),
                            int(math.floor(r.y1 / g)) + 1):
                cy = iy * g
                # distance from cell center to axis-aligned segment
                nx = min(max(cx, min(ax, bx)), max(ax, bx))
                ny = min(max(cy, min(ay, by)), max(ay, by))
                if math.hypot(cx - nx, cy - ny) <= half:
                    occ[layer][(ix, iy)] = True
                    if collect is not None:
                        collect.add((ix, iy))

    def setup(self, net, width):
        """Build occupancy + via-block grids for routing `net`."""
        occ = {"F": {}, "B": {}}
        viab = {}                     # cells where a via may not be placed
        viabocc = {"F": viab}
        hw = width / 2.0
        inf = CLEAR + hw
        vinf = VIA_DIA / 2 + CLEAR
        for (ref, name), pnet in self.netof.items():
            if pnet == net:
                continue
            for ax, ay, w, h, dr in self.shapes[(ref, name)]:
                r = Rect(ax - w / 2 - inf, ay - h / 2 - inf,
                         ax + w / 2 + inf, ay + h / 2 + inf)
                self._block_rect(occ, "F", r)
                if dr:    # plated/unplated hole -> both layers
                    self._block_rect(occ, "B", r)
                    # hole-edge clearance needs margin beyond the pad ring
                    # (min_hole_clearance is 0.25)
                    hm = dr[0] / 2 + 0.27 + hw
                    self._block_rect(
                        occ, "B", Rect(ax - max(w, dr[0]) / 2 - hm,
                                       ay - max(h, dr[1]) / 2 - hm,
                                       ax + max(w, dr[0]) / 2 + hm,
                                       ay + max(h, dr[1]) / 2 + hm))
                    self._block_rect(
                        occ, "F", Rect(ax - max(w, dr[0]) / 2 - hm,
                                       ay - max(h, dr[1]) / 2 - hm,
                                       ax + max(w, dr[0]) / 2 + hm,
                                       ay + max(h, dr[1]) / 2 + hm))
                vi = vinf
                if dr:
                    vi = max(vi, dr[0] / 2 + 0.25 + VIA_DRILL / 2)
                vr = Rect(ax - w / 2 - vi, ay - h / 2 - vi,
                          ax + w / 2 + vi, ay + h / 2 + vi)
                self._block_rect(viabocc, "F", vr)
        # edge clearance: cell centers must stay inside shrunk keepins
        g = GRID
        m = EDGE_CLEAR + hw
        vm = EDGE_CLEAR + VIA_DIA / 2
        shrunk = [Rect(r.x0 + m, r.y0 + m, r.x1 - m, r.y1 - m)
                  for r in self.keepin]
        vshrunk = [Rect(r.x0 + vm, r.y0 + vm, r.x1 - vm, r.y1 - vm)
                   for r in self.keepin]
        for ix in range(int(18 / g), int(90 / g) + 1):
            for iy in range(int(10 / g), int(80 / g) + 1):
                cx, cy = ix * g, iy * g
                if not any(r.inside(cx, cy) for r in shrunk):
                    occ["F"][(ix, iy)] = True
                    occ["B"][(ix, iy)] = True
                if not any(r.inside(cx, cy) for r in vshrunk):
                    viab[(ix, iy)] = True
        for k in self.keepouts:
            self._block_rect(occ, "F", k)
            self._block_rect(occ, "B", k)
            self._block_rect(viabocc, "F", k.inflate(VIA_DIA / 2))
        # foreign pad rects (uninflated): used to veto start/goal cells
        # where a `width`-wide track could not keep copper clearance
        self._frects = []
        for (ref, name), pnet in self.netof.items():
            if pnet == net:
                continue
            for ax, ay, w, h, dr in self.shapes[(ref, name)]:
                self._frects.append(Rect(ax - w / 2, ay - h / 2,
                                         ax + w / 2, ay + h / 2))
        # reserved fan-out stub lanes of other nets (e.g. the J3 zipper
        # rows): blocked even before those nets route, so tracks can only
        # ever cross a stub beyond its tip
        for (ref, name), pnet in self.netof.items():
            if pnet == net or not pnet or ref != "J3":
                continue
            end = self.stub_end((ref, name))
            if end:
                ax, ay = self.shapes[(ref, name)][0][:2]
                self._block_seg(occ, "F", ax, ay, end[0], end[1],
                                SIGNAL_W / 2 + CLEAR + hw)
                self._block_seg(viabocc, "F", ax, ay, end[0], end[1],
                                SIGNAL_W / 2 + CLEAR + VIA_DIA / 2)
        # already routed tracks/vias of OTHER nets (with provenance for
        # rip-up: src records which net owns each blocked cell)
        src = {}
        for (layer, ax, ay, bx, by, w, n) in self.segs:
            if n == net:
                continue
            mark = set()
            self._block_seg(occ, layer, ax, ay, bx, by,
                            w / 2 + CLEAR + hw, collect=mark)
            for c in mark:
                src[(layer, c[0], c[1])] = n
            self._block_seg(viabocc, "F", ax, ay, bx, by,
                            w / 2 + CLEAR + VIA_DIA / 2)
        for (vx, vy, n) in self.vias:
            if n == net:
                continue
            for L in ("F", "B"):
                mark = set()
                self._block_seg(occ, L, vx, vy, vx, vy,
                                VIA_DIA / 2 + CLEAR + hw, collect=mark)
                for c in mark:
                    src[(L, c[0], c[1])] = n
            self._block_seg(viabocc, "F", vx, vy, vx, vy,
                            VIA_DIA + CLEAR)
        return occ, viab, src

    def pad_cells(self, pp, width):
        """grid cells whose centers land inside one physical pad shape.

        pp is (ref, padname, shape_index): several *pads* may share a pad
        number (e.g. the two sides of a 4-pin button) — each is separate
        copper and each must be reached."""
        rp, si = pp
        ax, ay, w, h, dr = self.shapes[rp][si]
        out = set()
        for c in self._cells(Rect(ax - w / 2, ay - h / 2,
                                  ax + w / 2, ay + h / 2)):
            cx, cy = c[0] * GRID, c[1] * GRID
            if abs(cx - ax) <= w / 2 and abs(cy - ay) <= h / 2:
                out.add(c)
        if not out:
            out.add((int(round(ax / GRID)), int(round(ay / GRID))))
        return out

    def conn_cells(self, pp, width, occ):
        """pad cells where a `width` track may legally end/start: inside
        the pad, not already blocked by foreign copper, and far enough
        from neighbouring foreign pads to keep clearance (a wide track
        ending at a pad-edge cell would overhang onto the neighbour)."""
        need = CLEAR + width / 2.0
        out = set()
        for c in self.pad_cells(pp, width):
            if occ["F"].get(c):
                continue
            cx, cy = c[0] * GRID, c[1] * GRID
            bad = False
            for r in self._frects:
                nx = min(max(cx, r.x0), r.x1)
                ny = min(max(cy, r.y0), r.y1)
                if math.hypot(cx - nx, cy - ny) < need - 1e-9:
                    bad = True
                    break
            if not bad:
                out.add(c)
        return out

    # guaranteed fan-out corridors: a straight stub off the pad tip that
    # is emitted as real copper and reserved as start/goal cells, so the
    # fine-pitch escape cannot be walled in by neighbours' tracks.
    #
    # J3 (USB-C) has two interleaved pad rows.  Every pad gets an +x stub
    # whose tip sits at a column ordered by pad height: a track turning
    # away at its tip then only ever crosses stubs that end earlier, so
    # the escapes cannot wall each other in.  The lanes are blocked for
    # foreign nets in setup() before any routing happens.
    J3_STUB_X0 = 31.05
    J3_STUB_DX = 0.5

    def _j3_stubs(self):
        """(ax, ay) -> stub tip x for every routed J3 pad."""
        if hasattr(self, "_j3stubmap"):
            return self._j3stubmap
        pts = []
        for (ref, name), pnet in self.netof.items():
            if ref != "J3" or not pnet or pnet == "GND" or not name:
                continue
            for ax, ay, w, h, dr in self.shapes[(ref, name)]:
                pts.append((ax, ay))
        ys = sorted({round(ay, 2) for _, ay in pts})
        rank = {y: i for i, y in enumerate(ys)}
        self._j3stubmap = {
            (ax, round(ay, 2)): self.J3_STUB_X0 + self.J3_STUB_DX * rank[y]
            for ax, ay in pts for y in [round(ay, 2)]}
        return self._j3stubmap

    def stub_end(self, rp):
        if rp[0] != "J3":
            return None
        ax, ay = self.shapes[rp][0][:2]
        tip = self._j3_stubs().get((ax, round(ay, 2)))
        return (tip, ay) if tip else None

    def stub_cells(self, pp, width):
        """pad cells plus cells along the pad's escape stub (if any)."""
        rp, si = pp
        cells = set(self.pad_cells(pp, width))
        end = self.stub_end(rp) if si == 0 else None
        if end:
            ax, ay = self.shapes[rp][si][:2]
            iy = int(round(ay / GRID))
            ix0 = int(round(ax / GRID))
            ix1 = int(round(end[0] / GRID))
            cells.update((ix, iy) for ix in range(ix0, ix1 + 1))
        return cells

    def stub_only_cells(self, pp, width):
        """stub cells that lie outside the pad proper (for used-checks)."""
        rp, si = pp
        end = self.stub_end(rp) if si == 0 else None
        if not end:
            return set()
        ax, ay = self.shapes[rp][si][:2]
        iy = int(round(ay / GRID))
        ix0 = int(round(ax / GRID))
        ix1 = int(round(end[0] / GRID))
        return {(ix, iy) for ix in range(ix0, ix1 + 1)} - \
            self.pad_cells(pp, width)

    def astar(self, occ, viab, starts, goals, width):
        """4-conn A* + via transitions. Returns [(layer,ix,iy)...] or None.

        goals is a set of (layer, ix, iy) states: reaching a goal on the
        wrong layer does not count (an SMD pad exists only on F)."""
        goalset = set(goals)
        gx = sorted({g[1] for g in goals})
        gy = sorted({g[2] for g in goals})

        def heur(ix, iy):
            dx = min(abs(ix - a) for a in gx)
            dy = min(abs(iy - a) for a in gy)
            return (dx + dy) * GRID

        openq = [(heur(s[1], s[2]), 0.0, s) for s in starts]
        heapq.heapify(openq)
        best = {}
        came = {}
        exp = 0
        while openq and exp < MAX_ASTAR_EXPANSIONS:
            f, cost, state = heapq.heappop(openq)
            if state in best and best[state] <= cost:
                continue
            best[state] = cost
            layer, ix, iy = state
            exp += 1
            if state in goalset:
                path = [state]
                while state in came:
                    state = came[state]
                    path.append(state)
                return list(reversed(path))
            for dx_, dy_ in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = ix + dx_, iy + dy_
                if occ[layer].get((nx, ny)) and (layer, nx, ny) not in goalset:
                    continue
                ns = (layer, nx, ny)
                nc = cost + GRID
                if ns not in best or nc < best[ns] - 1e-9:
                    came[ns] = state
                    heapq.heappush(openq, (nc + heur(nx, ny), nc, ns))
            ol = "B" if layer == "F" else "F"
            if (ix, iy) not in viab and \
                    (not occ[ol].get((ix, iy)) or (ol, ix, iy) in goalset):
                ns = (ol, ix, iy)
                nc = cost + VIA_COST
                if ns not in best or nc < best[ns] - 1e-9:
                    came[ns] = state
                    heapq.heappush(openq, (nc + heur(ix, iy), nc, ns))
        return None

    def start_cells(self, pp, width, stubs=False, occ=None):
        """start states: pad cells on F, plus B for through-hole pads."""
        cells = (self.stub_cells(pp, width)
                 if stubs and self.stub_end(pp[0])
                 else (self.conn_cells(pp, width, occ) if occ is not None
                       else self.pad_cells(pp, width)))
        rp, si = pp
        out = [("F", ix, iy) for (ix, iy) in cells]
        if any(dr for *_, dr in self.shapes[rp]):
            out += [("B", ix, iy) for (ix, iy) in cells]
        return out

    def route(self, net):
        """Route one net -> list of emitted items, or None on failure."""
        width = WIDTHS.get(net, SIGNAL_W)
        # every physical pad shape is a separate target (button pads come
        # in same-numbered pairs that KiCad still wants copper between)
        pads = [(rp, si) for rp in self.pads_by_net.get(net, [])
                for si in range(len(self.shapes[rp]))]
        if len(pads) < 2:
            return []
        # connect pads greedily: nearest to centroid first, then nearest to tree
        pts = {pp: self.shapes[pp[0]][pp[1]][:2] for pp in pads}
        cx = sum(p[0] for p in pts.values()) / len(pts)
        cy = sum(p[1] for p in pts.values()) / len(pts)
        base = sorted(pads, key=lambda pp:
                      (pts[pp][0] - cx) ** 2 + (pts[pp][1] - cy) ** 2)
        for attempt in range(MAX_NET_ATTEMPTS):
            occ, viab, src = self.setup(net, width)
            self._occ = occ
            self._viab = viab
            self._src = src
            # each try uses a different seed order -> different tree,
            # different corridors claimed first (bounded: 4 tries)
            orders = (base, list(reversed(base)),
                      sorted(base, key=lambda pp: pts[pp][1]),
                      sorted(base, key=lambda pp: -pts[pp][1]))
            remaining = list(orders[attempt])
            tree = [remaining.pop(0)]
            # goal states: (layer,ix,iy) cells the new path may terminate
            # on.  SMD pads exist on F only; drilled pads and via cells
            # count on both layers.
            def pad_goals(pp):
                cells = (self.stub_cells(pp, width)
                         if self.stub_end(pp[0])
                         else self.conn_cells(pp, width, occ)
                         or self.pad_cells(pp, width))
                layers = ("F", "B") if self.shapes[pp[0]][pp[1]][4] \
                    else ("F",)
                return {(L,) + c for L in layers for c in cells}
            goals = set()
            viacells = set()
            goals |= pad_goals(tree[0])
            paths = []
            fail = None
            while remaining:
                remaining.sort(key=lambda pp: min(
                    (pts[pp][0] - pts[q][0]) ** 2 +
                    (pts[pp][1] - pts[q][1]) ** 2 for q in tree))
                pp = remaining[0]
                path = self.astar(occ, viab,
                                  self.start_cells(pp, width, stubs=True,
                                                   occ=occ)
                                  or self.start_cells(pp, width),
                                  goals, width)
                if path is None:
                    fail = pp
                    self.failinfo = (pp, net, width)
                    break
                paths.append(path)
                for i, (layer, ix, iy) in enumerate(path):
                    goals.add((layer, ix, iy))
                    if i and path[i - 1][0] != layer:
                        viacells.add((ix, iy))
                for vc in viacells:
                    goals.add(("F",) + vc)
                    goals.add(("B",) + vc)
                goals |= pad_goals(pp)
                tree.append(pp)
                remaining.pop(0)
            if fail is None:
                self.commit(net, width, paths)
                return paths
        return None

    def commit(self, net, width, paths):
        """Convert cell paths to segments/vias and record as obstacles."""
        padcell = {}
        used = set()
        for path in paths:
            for layer, ix, iy in path:
                used.add((ix, iy))
        for rp in self.pads_by_net.get(net, []):
            for si in range(len(self.shapes[rp])):
                pp = (rp, si)
                for c in self.pad_cells(pp, width):
                    padcell[c] = pp
                end = self.stub_end(rp) if si == 0 else None
                hit = used & self.stub_only_cells(pp, width)
                if end and hit:
                    # emit the stub only as far as the path actually uses
                    # it — a full-length stub past the path is a dangling
                    # spur
                    ax, ay = self.shapes[rp][si][:2]
                    fx = max(c[0] for c in hit) * GRID
                    self.emit_seg(net, "F", (ax / GRID, ay / GRID),
                                  (fx / GRID, ay / GRID), width)
        for path in paths:
            # anchor path endpoints to pad centres: grid cells near a small
            # pad edge may not geometrically touch the pad otherwise.  The
            # anchor endpoint is clamped inside the pad rect inset by the
            # track half-width so it cannot overhang onto a neighbour pad.
            for (layer, ix, iy) in (path[0], path[-1]):
                pp = padcell.get((ix, iy))
                if pp is None:
                    continue
                rp, si = pp
                ax, ay, w, h, dr = self.shapes[rp][si]
                hw = width / 2.0
                bx = min(max(ix * GRID, ax - w / 2 + hw),
                         ax + w / 2 - hw) if w > 2 * hw else ax
                by = min(max(iy * GRID, ay - h / 2 + hw),
                         ay + h / 2 - hw) if h > 2 * hw else ay
                own = {c for c, q in padcell.items() if q == pp}
                if self._anchor_clear(layer, ax, ay, bx, by, own):
                    self.emit_seg(net, layer, (ax / GRID, ay / GRID),
                                  (bx / GRID, by / GRID), width)
            # split into maximal straight runs on the same layer
            run_layer = path[0][0]
            run_start = (path[0][1], path[0][2])
            prev = (path[0][1], path[0][2])
            for i in range(1, len(path)):
                layer, ix, iy = path[i]
                if layer != run_layer:
                    # emit run + via at bend
                    self.emit_seg(net, run_layer, run_start, prev, width)
                    self.emit_via(net, prev)
                    run_layer = layer
                    run_start = prev
                else:
                    dx = ix - prev[0]
                    dy = iy - prev[1]
                    if i + 1 < len(path):
                        nx, ny = path[i + 1][1], path[i + 1][2]
                        if (nx - ix != dx) or (ny - iy != dy):
                            self.emit_seg(net, run_layer, run_start,
                                          (ix, iy), width)
                            run_start = (ix, iy)
                prev = (ix, iy)
            self.emit_seg(net, run_layer, run_start, prev, width)

    def _anchor_clear(self, layer, ax, ay, bx, by, owncells):
        """True if a straight anchor seg a->b only crosses free/own cells."""
        occ = getattr(self, "_occ", None)
        if occ is None:
            return True
        steps = max(1, int(math.hypot(bx - ax, by - ay) / (GRID * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            c = (int(round((ax + (bx - ax) * t) / GRID)),
                 int(round((ay + (by - ay) * t) / GRID)))
            if c not in owncells and occ[layer].get(c):
                return False
        return True

    def emit_seg(self, net, layer, a, b, width):
        if a == b:
            return
        x0, y0 = a[0] * GRID, a[1] * GRID
        x1, y1 = b[0] * GRID, b[1] * GRID
        self.segs.append((layer, x0, y0, x1, y1, width, net))

    def emit_via(self, net, cell):
        self.vias.append((cell[0] * GRID, cell[1] * GRID, net))


    def rip(self, net):
        """remove all committed copper of `net` (rip-up & reroute)."""
        self.segs = [s for s in self.segs if s[6] != net]
        self.vias = [v for v in self.vias if v[2] != net]

    def enclosing(self, net):
        """nets whose tracks wall in the last failed pad.

        Flood-fills the free space reachable from the failed pad (bounded
        to 6000 cells); every blocked boundary cell maps back to the net
        that owns it via the provenance table built in setup()."""
        if not hasattr(self, "failinfo"):
            return []
        rp, _, width = self.failinfo
        from collections import deque, Counter
        occ, viab, src = self._occ, self._viab, self._src
        seen = set(self.start_cells(rp, width, stubs=True))
        q = deque(seen)
        cnt = Counter()
        while q and len(seen) < 6000:
            layer, ix, iy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                c = (ix + dx, iy + dy)
                ns = (layer,) + c
                if ns in seen:
                    continue
                if occ[layer].get(c):
                    n = src.get((layer,) + c)
                    if n and n != net:
                        cnt[n] += 1
                    continue
                seen.add(ns)
                q.append(ns)
            ol = "B" if layer == "F" else "F"
            if (ix, iy) not in viab and not occ[ol].get((ix, iy)):
                ns = (ol, ix, iy)
                if ns not in seen:
                    seen.add(ns)
                    q.append(ns)
        return cnt


def place_gnd_vias(r):
    """Pre-place GND stitching vias next to GND pads before routing.

    Pour pockets isolate pads when a wall of tracks encloses them; a via
    placed while corridors are still open punches the pocket down to the
    B-side pour.  Via sites become occupancy for every later net."""
    occ, viab, src = r.setup("GND", 0.3)
    # hole-to-hole clearance applies even to same-net holes
    holes = [(ax, ay, dr[0]) for rp in r.pads_by_net.get("GND", [])
             for ax, ay, w, h, dr in r.shapes[rp] if dr]
    placed = []
    for rp in r.pads_by_net.get("GND", []):
        for si, (ax, ay, w, h, dr) in enumerate(r.shapes[rp]):
            if any((vx - ax) ** 2 + (vy - ay) ** 2 < 2.3 ** 2
                   for vx, vy, _ in placed):
                continue
            cx, cy = int(round(ax / GRID)), int(round(ay / GRID))
            site = None
            for rad in range(1, 14):
                ring = [(cx + dx, cy + dy)
                        for dx in range(-rad, rad + 1)
                        for dy in (-rad, rad)]
                ring += [(cx + dx, cy + dy)
                         for dy in range(-rad + 1, rad)
                         for dx in (-rad, rad)]
                for ix, iy in ring:
                    if (ix, iy) in viab:
                        continue
                    if occ["F"].get((ix, iy)) or occ["B"].get((ix, iy)):
                        continue
                    vx, vy = ix * GRID, iy * GRID
                    if any((vx - hx) ** 2 + (vy - hy) ** 2 <
                           (hd / 2 + VIA_DRILL / 2 + 0.26) ** 2
                           for hx, hy, hd in holes):
                        continue
                    site = (ix, iy)
                    break
                if site:
                    break
            if site:
                vx, vy = site[0] * GRID, site[1] * GRID
                r.vias.append((vx, vy, "GND"))
                placed.append((vx, vy, "GND"))
                # block the spot for the rest of this pre-pass
                r._block_seg(occ, "F", vx, vy, vx, vy,
                             VIA_DIA / 2 + CLEAR)
                r._block_seg(occ, "B", vx, vy, vx, vy,
                             VIA_DIA / 2 + CLEAR)
                viab[site] = True
    return placed


def prune_deadends(r):
    """Drop segments that dead-end in free space.

    A kept endpoint must terminate on a pad/via of its net or T into
    another of its net's tracks; anything else is a spur left over from
    rip-ups and aborted goals (DRC `track_dangling`)."""
    segs = r.segs
    pads = {}   # net -> list of (x0,y0,x1,y1, drilled)
    for rp, net in r.netof.items():
        for ax, ay, w, h, dr in r.shapes[rp]:
            pads.setdefault(net, []).append(
                (ax - w / 2, ay - h / 2, ax + w / 2, ay + h / 2,
                 bool(dr)))

    # union-find over seg endpoints: nodes joined by a seg (or touching
    # another seg mid-way) form one component; a component is alive iff
    # it holds a node on a pad or via of the net.
    parent = {}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    def union(a, b):
        parent[find(a)] = find(b)

    def key(x, y):
        return (round(x * 4), round(y * 4))

    def rebuild():
        parent.clear()
        # via cells exist as nodes on both layers and bridge them
        for vx, vy, vn in r.vias:
            for L in ("F", "B"):
                parent.setdefault((L, vn) + key(vx, vy),
                                  (L, vn) + key(vx, vy))
            union(("F", vn) + key(vx, vy), ("B", vn) + key(vx, vy))
        for (L, ax, ay, bx, by, w, n) in segs:
            for nd in (key(ax, ay), key(bx, by)):
                parent.setdefault((L, n) + nd, (L, n) + nd)
            union((L, n) + key(ax, ay), (L, n) + key(bx, by))
            # a seg endpoint at a via cell joins the via's node
            for vx, vy, vn in r.vias:
                if vn == n and (vx - ax) ** 2 + (vy - ay) ** 2 < 0.09:
                    union((L, n) + key(ax, ay), (L, n) + key(vx, vy))
                if vn == n and (vx - bx) ** 2 + (vy - by) ** 2 < 0.09:
                    union((L, n) + key(bx, by), (L, n) + key(vx, vy))
        # T-junctions: a node lying on another seg joins that seg's ends
        for (L, ax, ay, bx, by, w, n) in segs:
            for nd in list(parent):
                l2, n2, kx, ky = nd
                if l2 != L or n2 != n:
                    continue
                x, y = kx / 4.0, ky / 4.0
                dx, dy = bx - ax, by - ay
                l = dx * dx + dy * dy
                t = 0.0 if l == 0 else max(
                    0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l))
                nx, ny = ax + t * dx, ay + t * dy
                if (x - nx) ** 2 + (y - ny) ** 2 < 0.01:
                    union(nd, (L, n) + key(ax, ay))

    def on_pad_or_via(net, layer, x, y):
        for x0, y0, x1, y1, dr in pads.get(net, []):
            if (layer == "F" or dr) and \
                    x0 - 0.05 <= x <= x1 + 0.05 and \
                    y0 - 0.05 <= y <= y1 + 0.05:
                return True
        for vx, vy, vn in r.vias:
            if vn == net and (vx - x) ** 2 + (vy - y) ** 2 < 0.09:
                return True
        return False

    for _ in range(8):
        rebuild()
        alive = set()
        for nd in parent:
            l2, n2, kx, ky = nd
            if on_pad_or_via(n2, l2, kx / 4.0, ky / 4.0):
                alive.add(find(nd))
        if not alive:
            break
        keep = [s for s in segs
                if find((s[0], s[6]) + key(s[1], s[2])) in alive]
        segs[:] = keep
        # leaf-prune: inside a live component, drop segs whose endpoint
        # has degree 1 and sits on no pad/via (dead branches)
        deg = {}
        for s in segs:
            for nd in ((s[0], s[6]) + key(s[1], s[2]),
                       (s[0], s[6]) + key(s[3], s[4])):
                deg[nd] = deg.get(nd, 0) + 1
        drop = []
        for s in segs:
            for nd, (x, y) in (((s[0], s[6]) + key(s[1], s[2]),
                                (s[1], s[2])),
                               ((s[0], s[6]) + key(s[3], s[4]),
                                (s[3], s[4]))):
                if deg.get(nd) == 1 and \
                        not on_pad_or_via(s[6], s[0], x, y):
                    drop.append(s)
                    break
        if not drop:
            break
        for s in drop:
            segs.remove(s)
    return segs


MAX_RIPUPS = 8


def route_gnd(r):
    """Best-effort copper for GND pads the pours may strand.

    GND relies on the two pours, but dense routing cuts the F pour into
    pockets that wrap a pad with no via site left inside.  Those pads
    still get connected: a same-net path can dive to B.Cu (the pour
    cut-out there is free routing space), sneak under the foreign tracks
    that wall the pocket, and come up on the mainland.  Routed last so
    the copper lands on top of the settled signal layout; whatever the
    A* cannot reach the pours cover anyway.
    """
    net = "GND"
    width = 0.4
    pads = [(rp, si) for rp in r.pads_by_net.get(net, [])
            for si in range(len(r.shapes[rp]))]
    if len(pads) < 2:
        return
    occ, viab, src = r.setup(net, width)
    r._occ = occ
    # stacked drills are illegal even between same-net vias
    viabocc = {"F": viab}
    for (vx, vy, n) in r.vias:
        r._block_seg(viabocc, "F", vx, vy, vx, vy,
                     VIA_DIA / 2 + 0.25 + VIA_DRILL / 2)
    pts = {pp: r.shapes[pp[0]][pp[1]][:2] for pp in pads}
    cx = sum(p[0] for p in pts.values()) / len(pts)
    cy = sum(p[1] for p in pts.values()) / len(pts)
    order = sorted(pads, key=lambda pp:
                   (pts[pp][0] - cx) ** 2 + (pts[pp][1] - cy) ** 2)
    seed = order[0]
    layers = ("F", "B") if r.shapes[seed[0]][seed[1]][4] else ("F",)
    goals = {(L,) + c for L in layers
             for c in (r.conn_cells(seed, width, occ)
                       or r.pad_cells(seed, width))}
    # every pre-placed GND via is a goal on both layers
    for (vx, vy, n) in r.vias:
        if n == net:
            c = (int(round(vx / GRID)), int(round(vy / GRID)))
            goals.add(("F",) + c)
            goals.add(("B",) + c)
    ok = 0
    for pp in order[1:]:
        starts = (r.start_cells(pp, width, occ=occ)
                  or r.start_cells(pp, width))
        path = r.astar(occ, viab, starts, goals, width)
        if path is None:
            continue
        r.commit(net, width, [path])
        ok += 1
        for (layer, ix, iy) in path:
            goals.add((layer, ix, iy))
    print(f"  [router] GND copper: {ok}/{len(pads) - 1} pads routed")


def route_all():
    r = Router()
    gnd_vias = place_gnd_vias(r)
    print(f"  [router] {len(gnd_vias)} GND stitch vias pre-placed")
    order = [n for n in ORDER if n in r.pads_by_net] + \
            sorted(n for n in r.pads_by_net
                   if n not in ORDER and n != "GND")
    failed = []
    ripups = 0
    i = 0
    while i < len(order):
        net = order[i]
        i += 1
        if net == "GND":
            continue
        if r.route(net) is not None:
            continue
        if ripups < MAX_RIPUPS and hasattr(r, "failinfo"):
            cnt = r.enclosing(net)
            # rip up to 2 nets already routed that wall the pad in
            todo = [n for n, _ in cnt.most_common(2)
                    if any(s[6] == n for s in r.segs)]
            if todo:
                ripups += 1
                for n in todo:
                    r.rip(n)
                # failed net retries first into the freed space, ripped
                # nets re-route right behind it
                for n in reversed(todo):
                    order.insert(i, n)
                order.insert(i, net)
                continue
        failed.append(net)
    lift_vbus_wall(r)
    route_gnd(r)
    return r.segs, r.vias, failed


def lift_vbus_wall(r):
    """Lift VBUS_F's y=67.75 run to y=68.0.

    The run x59.75-62.5 @ y67.75 is the north wall of a sealed GND pour
    pocket around U5.3 — walled by U5's pad row, the LED_DIN stub, and
    BOOT/LED_DIN B-copper below, with no via site left inside.  Lifting
    the wall 0.25mm opens a pour neck between the LED_DIN stub and the
    moved run, letting the pocket merge into the stitched island north
    of it.  Deterministic coordinate surgery on the emitted segments.
    """
    def eq(a, b):
        return abs(a - b) < 0.01
    moved = []
    for s in r.segs:
        layer, x0, y0, x1, y1, w, n = s
        if n != "VBUS_F" or layer != "F":
            continue
        # the corner at (62.5,67.75) / (59.75,67.75)
        if eq(y0, 67.75) and eq(y1, 67.75) and \
                eq(min(x0, x1), 59.75) and eq(max(x0, x1), 62.5):
            moved.append(s)
        elif eq(x0, x1) and eq(x0, 62.5) and \
                eq(min(y0, y1), 67.75):
            moved.append(s)
        elif eq(x0, x1) and eq(x0, 59.75) and \
                eq(max(y0, y1), 67.75) and eq(min(y0, y1), 66.0):
            moved.append(s)
    if len(moved) != 3:
        print(f"  [router] vbus lift: matched {len(moved)} segs, skip")
        return
    for s in moved:
        r.segs.remove(s)
    w = moved[0][5]
    r.segs.append(("F", 62.5, 73.75, 62.5, 68.0, w, "VBUS_F"))
    r.segs.append(("F", 62.5, 68.0, 59.75, 68.0, w, "VBUS_F"))
    r.segs.append(("F", 59.75, 68.0, 59.75, 66.0, w, "VBUS_F"))
    print("  [router] vbus lift: wall moved 67.75 -> 68.0")
