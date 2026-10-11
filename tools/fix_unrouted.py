#!/usr/bin/env python3
"""Fallback: connect any remaining unconnected pads with straight tracks.

DRC-be-damned direct connection per the owner's instruction ("any
connection is fine"). If the two pads share a copper layer, draw the
segment there; otherwise drop a via on the source pad and finish on the
target pad's layer.

Run under KiCad's bundled python AFTER import_ses.py.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad", "keychain.kicad_pcb")

import pcbnew  # noqa: E402

MM = pcbnew.FromMM
POWER_NETS = {"VBUS", "+BATT", "/VBAT_SW", "+3V3", "/VCC", "GND"}


def pid(pad):
    return pad.GetParentFootprint().GetReferenceAsString() + "-" \
        + pad.GetNumber()


def pad_layers(pad):
    ls = pad.GetLayerSet()
    return [l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if ls.Contains(l)]


def link(board, a, b, width):
    la, lb = pad_layers(a), pad_layers(b)
    shared = [l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if l in la and l in lb]
    pa, pb = a.GetPosition(), b.GetPosition()
    if shared:
        layer = shared[0]
    else:
        layer = lb[0] if lb else pcbnew.F_Cu
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pa)
        via.SetWidth(MM(0.6))
        via.SetDrill(MM(0.3))
        via.SetNetCode(a.GetNetCode())
        board.Add(via)
    seg = pcbnew.PCB_TRACK(board)
    seg.SetStart(pa)
    seg.SetEnd(pb)
    seg.SetLayer(layer)
    seg.SetWidth(width)
    seg.SetNetCode(a.GetNetCode())
    board.Add(seg)


def unrouted_pads(board):
    conn = board.GetConnectivity()
    conn.Build(board)
    out = []
    for net in board.GetNetsByName().values():
        name = net.GetNetname()
        if name.startswith("unconnected-") or not name:
            continue
        pads = [p for p in board.GetPads()
                if p.GetNetCode() == net.GetNetCode()]
        if len(pads) < 2:
            continue
        con = {pid(i) for i in conn.GetConnectedItems(pads[0])
               if isinstance(i, pcbnew.PAD)}
        islands = [p for p in pads if pid(p) not in con]
        connected = [p for p in pads if pid(p) in con]
        if islands and connected:
            out.append((name, islands, connected))
        elif islands:
            out.append((name, islands[1:], islands[:1]))
    return out, conn.GetUnconnectedCount(False)


def main():
    board = pcbnew.LoadBoard(BOARD)
    for _ in range(40):
        todo, total = unrouted_pads(board)
        if not todo:
            break
        fixed = 0
        for name, islands, connected in todo:
            width = MM(0.4) if name in POWER_NETS else MM(0.2)
            for p in islands:
                dst = min(connected,
                          key=lambda q: (p.GetPosition()
                                         - q.GetPosition()).EuclideanNorm())
                link(board, p, dst, width)
                fixed += 1
        print("patched", fixed, "links this round;", total, "edges were open")
    pcbnew.SaveBoard(BOARD, board)
    _, total = unrouted_pads(board)
    print("UNROUTED:", total)


if __name__ == "__main__":
    main()
