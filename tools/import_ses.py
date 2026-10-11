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

    # drop any existing un-filled GND zones we own, keep the antenna keepout
    for z in list(board.Zones()):
        if z.GetNetCode() == gnd.GetNetCode() and not z.GetIsRuleArea():
            board.Remove(z)

    for layer_name in ("F.Cu", "B.Cu"):
        layer = board.GetLayerID(layer_name)
        zs = pcbnew.ZONE_SETTINGS()
        zs.m_NetCode = gnd.GetNetCode()
        zone = pcbnew.ZONE(board)
        zone.SetLayer(layer)
        zone.SetNetCode(gnd.GetNetCode())
        zone.SetZoneClearance(pcbnew.FromMM(0.25))
        zone.SetMinThickness(pcbnew.FromMM(0.2))
        zone.SetThermalReliefGap(pcbnew.FromMM(0.3))
        zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.3))
        zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        outline = zone.Outline()
        outline.NewOutline()
        outline.Append(bbox.GetLeft(), bbox.GetTop())
        outline.Append(bbox.GetRight(), bbox.GetTop())
        outline.Append(bbox.GetRight(), bbox.GetBottom())
        outline.Append(bbox.GetLeft(), bbox.GetBottom())
        board.Add(zone)

    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())

    pcbnew.SaveBoard(BOARD, board)

    # unrouted count: ratlines = missing pad-to-pad links
    conn = board.GetConnectivity()
    unrouted = 0
    details = []
    for net in board.GetNetsByName().values():
        if net.GetNetCode() <= 0 or net.GetNetname().startswith("unconnected-"):
            continue
        n = conn.GetUnconnectedCount(net.GetNetCode()) \
            if hasattr(conn, "GetUnconnectedCount") else None
        # fallback: count via ratsnest items per net
        details.append((net.GetNetname(), n))
    print("done. see unrouted via ratsnest check:")
    rats = board.GetConnectivity().GetRatsnestForNet \
        if hasattr(board.GetConnectivity(), "GetRatsnestForNet") else None
    print(details)


if __name__ == "__main__":
    main()
