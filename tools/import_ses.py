#!/usr/bin/env python3
"""Import SES routes, add GND zones on F+B, refill, save, report unrouted.

Run under KiCad's bundled python.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad", "keychain.kicad_pcb")
SES = os.path.join(ROOT, "kicad", "keychain.ses")

import pcbnew  # noqa: E402


def main():
    board = pcbnew.LoadBoard(BOARD)
    ok = pcbnew.ImportSpecctraSES(board, SES)
    print("SES import:", ok)

    gnd = board.FindNet("GND")
    bbox = board.GetBoardEdgesBoundingBox()

    for layer_name in ("F.Cu", "B.Cu"):
        zone = pcbnew.ZONE(board)
        zone.SetLayer(board.GetLayerID(layer_name))
        zone.SetNetCode(gnd.GetNetCode())
        zone.SetLocalClearance(pcbnew.FromMM(0.2))
        zone.SetMinThickness(pcbnew.FromMM(0.2))
        zone.SetThermalReliefGap(pcbnew.FromMM(0.3))
        zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.3))
        outline = zone.Outline()
        outline.NewOutline()
        outline.Append(bbox.GetLeft(), bbox.GetTop())
        outline.Append(bbox.GetRight(), bbox.GetTop())
        outline.Append(bbox.GetRight(), bbox.GetBottom())
        outline.Append(bbox.GetLeft(), bbox.GetBottom())
        board.Add(zone)

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(BOARD, board)

    conn = board.GetConnectivity()
    conn.Build(board)
    print("UNROUTED:", conn.GetUnconnectedCount(False))


if __name__ == "__main__":
    main()
