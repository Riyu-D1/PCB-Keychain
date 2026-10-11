#!/usr/bin/env python3
"""Placement-only board builder for the e-Paper keychain design.

Reads connectivity + footprints from the schematic's exported netlist
(kicad-cli sch export netlist) and symbol uuids from keychain.kicad_sch,
then emits a fresh keychain.kicad_pcb: Edge.Cuts, footprints with nets,
antenna keepout, keyring NPTH, board-level silk labels.  No tracks/zones.
"""

import math
import os
import re
import subprocess
import sys
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate as G  # _block, _top_kids, fmt, esc, U, xform, rot_pt, load_fp, OUTDIR, FPDIR

KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
OUTDIR = G.OUTDIR
PROJ = "keychain"
SCH = os.path.join(OUTDIR, PROJ + ".kicad_sch")
PCB = os.path.join(OUTDIR, PROJ + ".kicad_pcb")
NETLIST = "/tmp/keychain_epd.net"


# ---------------------------------------------------------------------------
# board geometry
# ---------------------------------------------------------------------------
BX0, BY0, BW, BH = 30.0, 30.0, 68.1, 37.9       # 68.1 x 37.9 mm, panel size
BX1, BY1 = BX0 + BW, BY0 + BH
CR = 2.0                                        # corner radius

# placement: ref -> (x, y, rot, side)
PLACEMENT = {
    # --- front: visible SK6812 at bottom edge --------------------------------
    "LED1": (49.5, 65.85, 0, "F"),
    "LED2": (93.0, 65.8, 0, "F"),
    # --- keyring hole (corner, non-plated) ----------------------------------
    "H1": (36.0, 36.0, 0, "B"),
    # --- left edge: power switch + LiPo JST ---------------------------------
    "SW1": (33.3, 44.2, 270, "B"),
    "J2": (32.4, 53.6, 90, "B"),
    # --- bottom-left edge: USB-C facing out ---------------------------------
    "J1": (39.0, 62.6, 180, "B"),
    "R1": (32.0, 64.6, 0, "B"),      # CC1 pull-down beside J1's CC1 pin
    "R2": (45.9, 64.6, 0, "B"),      # CC2 pull-down beside J1's CC2 pin
    "C1": (46.5, 49.0, 0, "B"),      # VBUS cap below U2 VDD pin
    # --- charger block ------------------------------------------------------
    "U2": (48.5, 45.5, 0, "B"),
    "R3": (46.5, 42.3, 0, "B"),      # PROG resistor above U2 PROG pin
    "C2": (50.5, 49.0, 0, "B"),      # +BATT cap below U2 VBAT pin
    # --- LDO near MCU --------------------------------------------------------
    "U3": (72.5, 46.0, 0, "B"),
    "C3": (76.3, 46.0, 0, "B"),      # VIN cap right of U3 input pins
    "C4": (68.7, 45.5, 0, "B"),      # VOUT cap left of U3 output pin
    # --- ESP32: antenna toward right edge ------------------------------------
    "U1": (86.85, 48.0, 270, "B"),
    # 3V3 caps C5/C6/C29 under U1's 3V3 pad, C7+R4 under the EN pad
    "C5": (85.9, 58.4, 0, "B"),
    "C6": (89.0, 56.3, 0, "B"),
    "C29": (89.0, 58.4, 0, "B"),
    "C7": (85.3, 56.3, 0, "B"),
    "R4": (82.0, 56.3, 0, "B"),
    "R8": (75.2, 51.5, 0, "B"),
    "R11": (77.0, 57.5, 0, "B"),
    # --- user buttons + back LED along top ----------------------------------
    "SW2": (50.5, 33.0, 0, "B"),
    "SW3": (58.5, 33.0, 0, "B"),
    "LED3": (43.5, 32.6, 0, "B"),
    "LED4": (89.0, 35.5, 0, "B"),
    # --- FPC connector, ribbon folds around bottom edge ---------------------
    "J3": (62.0, 65.3, 180, "B"),
    # --- e-paper drive block above the FPC ----------------------------------
    # rail decoupling rows (0603) in J3 pin order, left-to-right = p26 -> p3
    "C14": (67.6, 55.9, 0, "B"),     # p23 VGH
    "C15": (58.6, 55.9, 0, "B"),     # p25 VGL
    "C16": (61.6, 55.9, 0, "B"),     # p25 VGL
    "C17": (67.6, 58.6, 0, "B"),     # p17 VSL2
    "C18": (64.6, 58.6, 0, "B"),     # p18 VSLV
    "C19": (61.6, 58.6, 0, "B"),     # p19 VSL
    "C20": (58.6, 58.6, 0, "B"),     # p21 VSH2
    "C21": (70.6, 55.9, 0, "B"),     # p22 VSHV
    "C22": (64.6, 55.9, 0, "B"),     # p24 VSH
    "C23": (55.6, 55.9, 0, "B"),     # p26 VCOM
    "C24": (67.6, 61.3, 0, "B"),     # p3 TVCOM
    "C25": (64.6, 61.3, 0, "B"),     # p4 1V8
    "C26": (61.6, 61.3, 0, "B"),     # p7 VCC
    "C27": (58.6, 61.3, 0, "B"),     # p14 VCC
    "C28": (55.6, 61.3, 0, "B"),     # p15 VCC
    "R7": (55.6, 58.6, 0, "B"),      # p26 VCOM resistor
    # charge pump: inductor / switch / pump cap / diodes / sense Rs, left of J3
    "L1": (46.0, 52.3, 90, "B"),
    "Q1": (50.0, 52.3, 0, "B"),
    "C12": (53.7, 52.3, 0, "B"),
    "D1": (46.8, 56.8, 0, "B"),
    "D2": (51.55, 56.8, 0, "B"),
    "D3": (46.8, 60.6, 0, "B"),
    "R5": (51.7, 59.9, 0, "B"),
    "R6": (51.7, 62.4, 0, "B"),
    "Q2": (67.0, 41.0, 0, "B"),
    "Q3": (71.0, 41.0, 0, "B"),
    "R9": (57.0, 37.5, 0, "B"),
    "R10": (61.0, 37.5, 0, "B"),
    # VBAT_SW LED caps beside each LED's VDD pin, on that LED's side
    "C8": (51.3, 62.7, 0, "F"),      # LED1 VDD
    "C9": (94.5, 62.7, 0, "F"),      # LED2 VDD
    "C10": (39.0, 31.7, 0, "B"),     # LED3 VDD
    "C11": (88.6, 32.4, 0, "B"),     # LED4 VDD
}

# Reference text offsets: ref -> desired board-space (dx, dy) from the
# footprint anchor, converted to footprint-local coords at emit time.
REFOFF = {
    "J1": (5.5, -4.4),      # above-right of connector
    "J2": (7.9, 0.0),       # right of body
    "J3": (0.0, -1.5),      # inside connector outline, below top silk line
    "SW1": (4.2, 0.0),      # right of switch
    "LED3": (-2.3, 0.0),
    "LED4": (0.0, 3.3),
    "U1": (0.0, -8.5),      # above module
    "U2": (0.0, -4.5),      # above IC, clear of R3
    "R1": (0.0, -1.55),
    "R2": (0.0, -1.55),
    "R3": (0.0, -1.55),
    "C1": (0.0, -1.55),
    "C2": (0.0, -1.55),
    "C3": (0.0, -1.6),
    "C4": (0.0, 1.55),
    "R4": (-2.4, 0.0),
    "R8": (0.0, -1.5),
    "R11": (0.0, 1.5),
    "C5": (0.0, 1.55),
    "C6": (2.6, 1.55),
    "C29": (0.0, 1.55),
    "C7": (-1.8, 1.55),
    "C8": (0.0, -1.5),
    "C10": (0.0, 1.5),
    "C11": (2.4, 0.0),
    "C9": (0.0, -1.5),
    "L1": (0.0, -2.0),
    "R5": (0.0, 1.4),
    "R6": (0.0, 1.5),
    "R7": (0.0, -1.55),
    "R9": (0.0, 1.4),
    "R10": (0.0, 1.4),
}
for _r in ("C14", "C15", "C16", "C17", "C18", "C19", "C20", "C21",
           "C22", "C23", "C24", "C25", "C26", "C27", "C28"):
    REFOFF[_r] = (0.0, -1.55)
REFOFF["C12"] = (0.0, -1.5)
for _r in ("D1", "D2", "D3"):
    REFOFF[_r] = (0.0, -1.6)
REFOFF["D1"] = (0.0, -2.0)
REFOFF["Q1"] = (1.5, 2.7)
REFOFF["Q2"] = (-0.6, 2.3)
REFOFF["Q3"] = (0.0, -2.3)

# refs hidden instead of moved (covered by board-level labels)
HIDE_REFS = {"SW2", "SW3"}

# extra footprints with no schematic symbol (added as board items only)
EXTRA_FOOTPRINTS = {
    "H1": ("MountingHole:MountingHole_3.2mm_M3", "Keyring"),
}

# antenna keepout: module -y region mapped to +x at rot270 -> right edge
KEEPOUTS = [(91.6, 40.8, BX1, 55.2)]


# ---------------------------------------------------------------------------
# netlist + schematic parsing
# ---------------------------------------------------------------------------

def export_netlist():
    subprocess.run([KCLI, "sch", "export", "netlist",
                    "--output", NETLIST, SCH], check=True)


def parse_netlist(path):
    data = open(path).read()
    st = data.index("(components")
    compblk = G._block(data, st)
    comps = {}   # ref -> (footprint, value, libpart)
    for m in re.finditer(r'\(comp\b', compblk):
        b = G._block(compblk, m.start())
        ref = re.search(r'\(ref "([^"]+)"\)', b).group(1)
        val = re.search(r'\(value "([^"]*)"\)', b)
        fp = re.search(r'\(footprint "([^"]*)"\)', b)
        lp = re.search(r'\(libsource \(lib "([^"]+)"\) \(part "([^"]+)"', b)
        comps[ref] = (fp.group(1) if fp else "",
                      val.group(1) if val else "",
                      (lp.group(1) + ":" + lp.group(2)) if lp else "")
    st = data.index("(nets")
    netblk = G._block(data, st)
    nets = {}    # name -> [(ref, pin)]
    for m in re.finditer(r'\(net\b', netblk[5:]):
        b = G._block(netblk, 5 + m.start())
        name = re.search(r'\(name "([^"]*)"\)', b).group(1)
        nodes = re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"', b)
        nets[name] = nodes
    return comps, nets


def symbol_uuids(path):
    """ref -> (symbol uuid, sheet uuid) from the schematic."""
    data = open(path).read()
    out = {}
    for m in re.finditer(r'\n\t\(symbol\n', data):
        b = G._block(data, m.start() + 1)
        um = re.search(r'\n\t\t\(uuid "([^"]+)"\)', b)
        im = re.search(r'\(instances\s*\(project "[^"]+"\s*'
                       r'\(path "([^"]+)"\s*\(reference "([^"]+)"', b)
        if um and im:
            out[im.group(2)] = (um.group(1), im.group(1))
    return out


# ---------------------------------------------------------------------------
# board emission
# ---------------------------------------------------------------------------

def build_pcb():
    comps, nets = parse_netlist(NETLIST)
    symu = symbol_uuids(SCH)

    pinnet = {}
    for name, nodes in nets.items():
        for rp in nodes:
            pinnet[rp] = name
    netnames = sorted(nets)
    netid = {n: i + 1 for i, n in enumerate(netnames)}

    missing = [r for r in comps if r not in PLACEMENT]
    extra = [r for r in PLACEMENT if r not in comps
             and r not in EXTRA_FOOTPRINTS]
    assert not missing, f"unplaced components: {missing}"
    assert not extra, f"placed but not in netlist: {extra}"

    out = ['(kicad_pcb', '\t(version 20260206)', '\t(generator "pcbnew")',
           '\t(generator_version "10.0")',
           '\t(general', '\t\t(thickness 1.6)', '\t\t(legacy_teardrops no)',
           '\t)', '\t(paper "A4")',
           '\t(layers',
           '\t\t(0 "F.Cu" signal)', '\t\t(2 "B.Cu" signal)',
           '\t\t(9 "F.Adhes" user "F.Adhesive")',
           '\t\t(11 "B.Adhes" user "B.Adhesive")',
           '\t\t(13 "F.Paste" user)', '\t\t(15 "B.Paste" user)',
           '\t\t(5 "F.SilkS" user "F.Silkscreen")',
           '\t\t(7 "B.SilkS" user "B.Silkscreen")',
           '\t\t(1 "F.Mask" user)', '\t\t(3 "B.Mask" user)',
           '\t\t(17 "Dwgs.User" user "User.Drawings")',
           '\t\t(19 "Cmts.User" user "User.Comments")',
           '\t\t(21 "Eco1.User" user "User.Eco1")',
           '\t\t(23 "Eco2.User" user "User.Eco2")',
           '\t\t(25 "Edge.Cuts" user)', '\t\t(27 "Margin" user)',
           '\t\t(31 "F.CrtYd" user "F.Courtyard")',
           '\t\t(29 "B.CrtYd" user "B.Courtyard")', '\t)',
           '\t(setup',
           '\t\t(pad_to_mask_clearance 0)',
           '\t\t(allow_soldermask_bridges_in_footprints no)',
           '\t\t(tenting', '\t\t\t(front yes)', '\t\t\t(back yes)', '\t\t)',
           '\t\t(covering', '\t\t\t(front no)', '\t\t\t(back no)', '\t\t)',
           '\t\t(plugging', '\t\t\t(front no)', '\t\t\t(back no)', '\t\t)',
           '\t\t(capping no)', '\t\t(filling no)',
           '\t\t(pcbplotparams',
           '\t\t\t(layerselection 0x00000000_00000000_00000000_000000a5)',
           '\t\t\t(plot_on_all_layers_selection 0x00000000_00000000_00000000_00000000)',
           '\t\t\t(disableapertmacros no)', '\t\t\t(usegerberextensions yes)',
           '\t\t\t(usegerberattributes no)', '\t\t\t(usegerberadvancedattributes no)',
           '\t\t\t(creategerberjobfile no)', '\t\t\t(dashed_line_dash_ratio 12)',
           '\t\t\t(dashed_line_gap_ratio 3)', '\t\t\t(svgprecision 6)',
           '\t\t\t(plotframeref no)', '\t\t\t(mode 1)', '\t\t\t(useauxorigin no)',
           '\t\t\t(pdf_front_fp_property_popups yes)',
           '\t\t\t(pdf_back_fp_property_popups yes)', '\t\t\t(pdf_metadata yes)',
           '\t\t\t(pdf_single_document no)', '\t\t\t(dxfpolygonmode yes)',
           '\t\t\t(dxfimperialunits yes)', '\t\t\t(dxfusepcbnewfont yes)',
           '\t\t\t(psnegative no)', '\t\t\t(psa4output no)',
           '\t\t\t(plot_black_and_white yes)', '\t\t\t(plotinvisibletext no)',
           '\t\t\t(sketchpadsonfab no)', '\t\t\t(plotpadnumbers no)',
           '\t\t\t(hidednponfab no)', '\t\t\t(sketchdnponfab yes)',
           '\t\t\t(crossoutdnponfab yes)', '\t\t\t(subtractmaskfromsilk no)',
           '\t\t\t(outputformat 1)', '\t\t\t(mirror no)', '\t\t\t(drillshape 1)',
           '\t\t\t(scaleselection 1)', '\t\t\t(outputdirectory "")',
           '\t\t)', '\t)']

    out.append('\t(net 0 "")')
    for n in netnames:
        out.append(f'\t(net {netid[n]} "{G.esc(n)}")')

    # footprints -----------------------------------------------------------
    refs = list(comps) + list(EXTRA_FOOTPRINTS)
    for ref in refs:
        if ref in EXTRA_FOOTPRINTS:
            fp, value = EXTRA_FOOTPRINTS[ref]
        else:
            fp, value, _lp = comps[ref]
        x, y, rot, side = PLACEMENT[ref]
        body = G.load_fp(fp).rstrip()
        body = re.sub(r'\(footprint "[^"]+"', f'(footprint "{fp}"',
                      body, count=1)
        if ref in symu:
            su, sheetp = symu[ref]
            path = f'{sheetp}/{su}'
        else:
            sheetp = '/9f6d5e12-7b33-4c1d-9a01-0000000000aa'
            path = sheetp
        # B-side footprints are emitted unflipped (F.Cu) with negated rot
        # so that pcbnew's horizontal Flip() lands them in the orientation
        # G.xform intends; finish_board.py performs the flip.
        frot = rot if side == "F" else (-rot) % 360
        ins = (f'\n\t\t(uuid "{G.U("fp:" + ref)}")'
               f'\n\t\t(at {G.fmt(x)} {G.fmt(y)} {G.fmt(frot)})'
               f'\n\t\t(path "{path}")'
               '\n\t\t(sheetname "/")'
               f'\n\t\t(sheetfile "{PROJ}.kicad_sch")')
        lm = re.search(r'\(layer "?[FB]\.Cu"?\)', body)
        j = body.index(')', lm.start())
        body = body[:j + 1] + ins + body[j + 1:]
        parts = []
        last = 0
        for m in re.finditer(r'\(pad "([^"]*)"|(\(pad (\S+))', body):
            st = m.start() if m.group(1) is not None else m.start(2)
            pname = m.group(1) if m.group(1) is not None else m.group(3)
            pb = G._block(body, st)
            orig_len = len(pb)
            # pad `at` angle is board-absolute in .kicad_pcb: must add the
            # footprint rotation (frot) to the library-local pad angle.
            at = re.search(r'\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', pb)
            if at:
                na = (float(at.group(3) or 0) + frot) % 360
                pb = (pb[:at.start()] +
                      f'(at {at.group(1)} {at.group(2)} {G.fmt(na)})' +
                      pb[at.end():])
            net = pinnet.get((ref, pname))
            nb = (f'\n\t\t\t(net {netid[net]} "{G.esc(net)}")' if net else "")
            parts.append(body[last:st])
            parts.append(pb.rstrip()[:-1] + nb + ")")
            last = st + orig_len
        parts.append(body[last:])
        body = "".join(parts)
        body = re.sub(r'\(property "Reference" "[^"]*"',
                      f'(property "Reference" "{ref}"', body, count=1)
        body = re.sub(r'\(property "Value" "[^"]*"',
                      f'(property "Value" "{G.esc(value)}"', body, count=1)
        # rename texts; font sizing/position/hiding of the Reference is
        # done in board space by finish_board.py (after pcbnew flips the
        # B-side footprints into canonical form).
        rm = re.search(r'\(property "Reference"', body)
        if rm is None:
            rm = re.search(r'\(fp_text reference\b', body)
        if rm and '(fp_text' in G._block(body, rm.start()):
            ob = G._block(body, rm.start())
            rblk = re.sub(r'\(fp_text reference "?[^"\s)]*"?',
                          f'(fp_text reference "{ref}"', ob)
            body = body[:rm.start()] + rblk + body[rm.start() + len(ob):]
        out.append("\t" + body.replace("\n", "\n\t"))

    # Edge.Cuts: rounded rectangle -----------------------------------------
    def e_line(x1, y1, x2, y2):
        out.append(f'\t(gr_line (start {G.fmt(x1)} {G.fmt(y1)}) '
                   f'(end {G.fmt(x2)} {G.fmt(y2)}) '
                   '(stroke (width 0.1) (type solid)) '
                   f'(layer "Edge.Cuts") (uuid "{G.U("ec")}"))')

    def e_arc(x1, y1, mx, my, x2, y2):
        out.append(f'\t(gr_arc (start {G.fmt(x1)} {G.fmt(y1)}) '
                   f'(mid {G.fmt(mx)} {G.fmt(my)}) (end {G.fmt(x2)} {G.fmt(y2)}) '
                   '(stroke (width 0.1) (type solid)) '
                   f'(layer "Edge.Cuts") (uuid "{G.U("ec")}"))')

    d = CR * (1 - math.sqrt(2) / 2)
    e_line(BX0 + CR, BY0, BX1 - CR, BY0)
    e_line(BX1, BY0 + CR, BX1, BY1 - CR)
    e_line(BX1 - CR, BY1, BX0 + CR, BY1)
    e_line(BX0, BY1 - CR, BX0, BY0 + CR)
    e_arc(BX0 + CR, BY0, BX0 + d, BY0 + d, BX0, BY0 + CR)
    e_arc(BX1, BY0 + CR, BX1 - d, BY0 + d, BX1 - CR, BY0)
    e_arc(BX1 - CR, BY1, BX1 - d, BY1 - d, BX1, BY1 - CR)
    e_arc(BX0, BY1 - CR, BX0 + d, BY1 - d, BX0 + CR, BY1)

    # antenna keepout -------------------------------------------------------
    for i, (kx0, ky0, kx1, ky1) in enumerate(KEEPOUTS):
        out.append(
            f'\t(zone (layers "F.Cu" "B.Cu") (uuid "{G.U("ko:" + str(i))}") '
            '(hatch edge 0.5) '
            '(connect_pads (clearance 0)) '
            '(min_thickness 0.25) '
            '(keepout (tracks not_allowed) (vias not_allowed) '
            '(pads allowed) (copperpour not_allowed) '
            '(footprints allowed)) '
            '(fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) '
            f'(polygon (pts (xy {G.fmt(kx0)} {G.fmt(ky0)}) '
            f'(xy {G.fmt(kx1)} {G.fmt(ky0)}) (xy {G.fmt(kx1)} {G.fmt(ky1)}) '
            f'(xy {G.fmt(kx0)} {G.fmt(ky1)}))))')

    # silkscreen ------------------------------------------------------------
    def silk(txt, x, y, rot=0, size=0.9, side="B"):
        layer = "F.SilkS" if side == "F" else "B.SilkS"
        jus = "" if side == "F" else " (justify mirror)"
        out.append(f'\t(gr_text "{txt}" (at {G.fmt(x)} {G.fmt(y)} {rot}) '
                   f'(layer "{layer}") '
                   f'(uuid "{G.U("silk")}") '
                   f'(effects (font (size {size} {size}) '
                   f'(thickness {round(size / 6, 3)})){jus}))')

    silk("PCB KEYCHAIN", 81.0, 32.4, 0, 1.1)
    silk("rev B - e-Paper", 81.0, 34.2, 0, 0.8)
    silk("BOOT", 50.5, 31.4, 0, 0.8)
    silk("USER", 58.5, 31.4, 0, 0.8)

    out.append(')')
    return "\n".join(out) + "\n"


def main():
    export_netlist()
    pcb = build_pcb()
    with open(PCB, "w") as f:
        f.write(pcb)


if __name__ == "__main__":
    main()
