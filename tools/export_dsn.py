#!/usr/bin/env python3
"""Set net classes then export Specctra DSN for freerouting.

Run under KiCad's bundled python.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad", "keychain.kicad_pcb")
DSN = os.path.join(ROOT, "kicad", "keychain.dsn")

import pcbnew  # noqa: E402

MM = pcbnew.FromMM

POWER_NETS = {"VBUS", "+BATT", "/VBAT_SW", "+3V3", "/VCC", "GND"}


def main():
    board = pcbnew.LoadBoard(BOARD)
    ds = board.GetDesignSettings()
    ncs = ds.GetNetClasses()

    dflt = ncs.GetDefault()
    dflt.SetTrackWidth(MM(0.2))
    dflt.SetViaDiameter(MM(0.6))
    dflt.SetViaDrill(MM(0.3))
    dflt.SetuViaDiameter(MM(0.6))
    dflt.SetuViaDrill(MM(0.3))
    dflt.SetClearance(MM(0.15))

    pwr = pcbnew.NETCLASS("PWR")
    pwr.SetTrackWidth(MM(0.4))
    pwr.SetViaDiameter(MM(0.6))
    pwr.SetViaDrill(MM(0.3))
    pwr.SetuViaDiameter(MM(0.6))
    pwr.SetuViaDrill(MM(0.3))
    pwr.SetClearance(MM(0.2))
    for name in POWER_NETS:
        pwr.Add(name)
    ncs.Add(pwr)

    nets = {n.GetNetname(): n.GetNetClass().GetName()
            for n in board.GetNetsByName().values()}
    print("nets:")
    for k in sorted(nets):
        print("  %-40s %s" % (k, nets[k]))

    pcbnew.SaveBoard(BOARD, board)
    ok = pcbnew.ExportSpecctraDSN(board, DSN)
    print("export:", ok, DSN)


if __name__ == "__main__":
    main()
