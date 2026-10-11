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
    "LED_D0", "LED_C1", "LED_C2", "LED_C3", "LED_DIN",
    "BOOST_SW", "LED_FB", "BOOST_EN",
    "USB_DP_CONN", "USB_DM_CONN", "CC1", "CC2",
    "CHG_LED", "PROG", "PWR_LED", "BAT_ADC",
    "+3V3", "+5V", "VBAT_SYS", "VBAT", "VBUS_F", "VBUS",
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

    def astar(self, occ, starts, goals, width):
        """4-conn A* + via transitions. Returns [(layer,ix,iy)...] or None."""
        goalset = set(goals)
        gx = {}
        gy = {}
        for g in goals:
            gx.setdefault(g[0], g[0])
            gy.setdefault(g[1], g[1])
        gx = sorted(gx); gy = sorted(gy)

        def heur(ix, iy):
            dx = min(abs(ix - a) for a in gx)
            dy = min(abs(iy - a) for a in gy)
            return (dx + dy) * GRID

        openq = [(heur(*s), 0.0, ("F", s[0], s[1])) for s in starts]
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
            if (ix, iy) in goalset:
                path = [state]
                while state in came:
                    state = came[state]
                    path.append(state)
                return list(reversed(path))
            for dx_, dy_ in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = ix + dx_, iy + dy_
                if occ[layer].get((nx, ny)) and (nx, ny) not in goalset:
                    continue
                ns = (layer, nx, ny)
                nc = cost + GRID
                if ns not in best or nc < best[ns] - 1e-9:
                    came[ns] = state
                    heapq.heappush(openq, (nc + heur(nx, ny), nc, ns))
            ol = "B" if layer == "F" else "F"
            if not occ[ol].get((ix, iy)) or (ix, iy) in goalset:
                ns = (ol, ix, iy)
                nc = cost + VIA_COST
                if ns not in best or nc < best[ns] - 1e-9:
                    came[ns] = state
                    heapq.heappush(openq, (nc + heur(ix, iy), nc, ns))
        return None

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
        remaining = sorted(pads, key=lambda rp:
                           (pts[rp][0] - cx) ** 2 + (pts[rp][1] - cy) ** 2)
        for attempt in range(MAX_NET_ATTEMPTS):
            occ = self.setup(net, width)
            tree = [remaining.pop(0)]
            treecells = set(self.pad_cells(tree[0], width))
            paths = []
            fail = None
            while remaining:
                remaining.sort(key=lambda rp: min(
                    (pts[rp][0] - pts[q][0]) ** 2 +
                    (pts[rp][1] - pts[q][1]) ** 2 for q in tree))
                rp = remaining[0]
                goals = set(treecells)
                goals |= set()
                for q in tree:
                    goals |= self.pad_cells(q, width)
                starts = self.pad_cells(rp, width)
                path = self.astar(occ, list(starts), list(goals), width)
                if path is None:
                    fail = rp
                    break
                paths.append(path)
                for (layer, ix, iy) in path:
                    treecells.add((ix, iy))
                tree.append(rp)
                remaining.pop(0)
            if fail is None:
                self.commit(net, width, paths)
                return paths
            remaining = sorted(pads, key=lambda rp:
                               (pts[rp][0] - cx) ** 2 + (pts[rp][1] - cy) ** 2)
        return None

    def commit(self, net, width, paths):
        """Convert cell paths to segments/vias and record as obstacles."""
        for path in paths:
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
