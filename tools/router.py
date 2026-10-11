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
MAX_NET_ATTEMPTS = 2             # retry budget per net

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
    "USB_DM_CONN", "USB_DP_CONN", "CC1", "CC2",
    "VBUS", "VBUS_F",
    "LED_D0", "LED_C1", "LED_C2", "LED_C3", "LED_DIN",
    "BOOST_SW", "LED_FB", "BOOST_EN",
    "CHG_LED", "PROG", "PWR_LED", "BAT_ADC",
    "+3V3", "+5V", "VBAT_SYS", "VBAT",
    "LED_DAT_3V", "USB_DP", "USB_DM",
    "OLED_SCK", "OLED_MOSI", "OLED_RES", "OLED_DC", "OLED_CS",
    "USER1", "USER2", "BOOT", "EN", "CHG_STAT",
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

    def _block_seg(self, occ, layer, ax, ay, bx, by, half):
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
                    hm = dr[0] / 2 + 0.35 + hw
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
        # already routed tracks/vias of OTHER nets
        for (layer, ax, ay, bx, by, w, n) in self.segs:
            if n == net:
                continue
            self._block_seg(occ, layer, ax, ay, bx, by,
                            w / 2 + CLEAR + hw)
            self._block_seg(viabocc, "F", ax, ay, bx, by,
                            w / 2 + CLEAR + VIA_DIA / 2)
        for (vx, vy, n) in self.vias:
            if n == net:
                continue
            self._block_seg(occ, "F", vx, vy, vx, vy,
                            VIA_DIA / 2 + CLEAR + hw)
            self._block_seg(occ, "B", vx, vy, vx, vy,
                            VIA_DIA / 2 + CLEAR + hw)
            self._block_seg(viabocc, "F", vx, vy, vx, vy,
                            VIA_DIA + CLEAR)
        return occ, viab

    def pad_cells(self, rp, width):
        """grid cells whose centers land inside a pad (own-net targets)."""
        out = set()
        for ax, ay, w, h, dr in self.shapes[rp]:
            for c in self._cells(Rect(ax - w / 2, ay - h / 2,
                                      ax + w / 2, ay + h / 2)):
                cx, cy = c[0] * GRID, c[1] * GRID
                if abs(cx - ax) <= w / 2 and abs(cy - ay) <= h / 2:
                    out.add(c)
            if not out:
                out.add((int(round(ax / GRID)), int(round(ay / GRID))))
        return out

    # guaranteed fan-out corridors: a straight stub off the pad tip that
    # is emitted as real copper and reserved as start/goal cells, so the
    # fine-pitch escape cannot be walled in by neighbours' tracks
    STUB_LEN = {"J3": 1.0}     # J3 zipper pads escape in +x

    def stub_end(self, rp):
        if rp[0] not in self.STUB_LEN:
            return None
        ax, ay = self.shapes[rp][0][:2]
        return (ax + self.STUB_LEN[rp[0]], ay)

    def stub_cells(self, rp, width):
        """pad cells plus cells along the pad's escape stub (if any)."""
        cells = set(self.pad_cells(rp, width))
        end = self.stub_end(rp)
        if end:
            ax, ay = self.shapes[rp][0][:2]
            iy = int(round(ay / GRID))
            ix0 = int(round(ax / GRID))
            ix1 = int(round(end[0] / GRID))
            cells.update((ix, iy) for ix in range(ix0, ix1 + 1))
        return cells

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

    def start_cells(self, rp, width, stubs=False):
        """start states: pad cells on F, plus B for through-hole pads."""
        cells = (self.stub_cells(rp, width) if stubs
                 else self.pad_cells(rp, width))
        out = [("F", ix, iy) for (ix, iy) in cells]
        if any(dr for *_, dr in self.shapes[rp]):
            out += [("B", ix, iy) for (ix, iy) in cells]
        return out

    def route(self, net):
        """Route one net -> list of emitted items, or None on failure."""
        width = WIDTHS.get(net, SIGNAL_W)
        pads = self.pads_by_net.get(net, [])
        if len(pads) < 2:
            return []
        # connect pads greedily: nearest to centroid first, then nearest to tree
        pts = {rp: self.shapes[rp][0][:2] for rp in pads}
        cx = sum(p[0] for p in pts.values()) / len(pts)
        cy = sum(p[1] for p in pts.values()) / len(pts)
        base = sorted(pads, key=lambda rp:
                      (pts[rp][0] - cx) ** 2 + (pts[rp][1] - cy) ** 2)
        for attempt in range(MAX_NET_ATTEMPTS):
            occ, viab = self.setup(net, width)
            self._occ = occ
            # second try: reversed seed order -> different tree, different
            # corridors claimed first
            remaining = list(base if attempt == 0 else reversed(base))
            tree = [remaining.pop(0)]
            # goal states: (layer,ix,iy) cells the new path may terminate
            # on.  SMD pads exist on F only; drilled pads and via cells
            # count on both layers.
            goals = set()
            viacells = set()
            for c in self.stub_cells(tree[0], width):
                layers = ("F", "B") if any(
                    dr for *_, dr in self.shapes[tree[0]]) else ("F",)
                for L in layers:
                    goals.add((L,) + c)
            paths = []
            fail = None
            while remaining:
                remaining.sort(key=lambda rp: min(
                    (pts[rp][0] - pts[q][0]) ** 2 +
                    (pts[rp][1] - pts[q][1]) ** 2 for q in tree))
                rp = remaining[0]
                path = self.astar(occ, viab,
                                  self.start_cells(rp, width, stubs=True),
                                  goals, width)
                if path is None:
                    fail = rp
                    break
                paths.append(path)
                for i, (layer, ix, iy) in enumerate(path):
                    goals.add((layer, ix, iy))
                    if i and path[i - 1][0] != layer:
                        viacells.add((ix, iy))
                for vc in viacells:
                    goals.add(("F",) + vc)
                    goals.add(("B",) + vc)
                q = rp
                for c in self.stub_cells(q, width):
                    layers = ("F", "B") if any(
                        dr for *_, dr in self.shapes[q]) else ("F",)
                    for L in layers:
                        goals.add((L,) + c)
                tree.append(rp)
                remaining.pop(0)
            if fail is None:
                self.commit(net, width, paths)
                return paths
        return None

    def commit(self, net, width, paths):
        """Convert cell paths to segments/vias and record as obstacles."""
        padcell = {}
        for rp in self.pads_by_net.get(net, []):
            for c in self.pad_cells(rp, width):
                padcell[c] = rp
            end = self.stub_end(rp)
            if end:
                ax, ay = self.shapes[rp][0][:2]
                self.emit_seg(net, "F", (ax / GRID, ay / GRID),
                              (end[0] / GRID, end[1] / GRID), width)
        for path in paths:
            # anchor path endpoints to pad centres: grid cells near a small
            # pad edge may not geometrically touch the pad otherwise.  The
            # anchor is only emitted when its straight run is clear of
            # foreign obstacles.
            for (layer, ix, iy) in (path[0], path[-1]):
                rp = padcell.get((ix, iy))
                if rp is None:
                    continue
                ax, ay = self.shapes[rp][0][:2]
                own = {c for c, q in padcell.items() if q == rp}
                if self._anchor_clear(layer, ax, ay, ix * GRID, iy * GRID,
                                      own):
                    self.emit_seg(net, layer, (ax / GRID, ay / GRID),
                                  (ix, iy), width)
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


def route_all():
    r = Router()
    order = [n for n in ORDER if n in r.pads_by_net] + \
            sorted(n for n in r.pads_by_net
                   if n not in ORDER and n != "GND")
    failed = []
    for net in order:
        if net == "GND":
            continue
        res = r.route(net)
        if res is None:
            failed.append(net)
    return r.segs, r.vias, failed
