#!/usr/bin/env python3
"""Refill all zones in the generated board, in place.

Usage (KiCad-bundled python):
    .../Python.framework/Versions/3.9/bin/python3.9 tools/fill_zones.py
"""
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad", "keychain.kicad_pcb")

sys.path.insert(0, "/Applications/KiCad/KiCad.app/Contents/Frameworks/"
                 "Python.framework/Versions/3.9/lib/python3.9/site-packages")
import pcbnew  # noqa: E402


def main():
    board = pcbnew.LoadBoard(BOARD)
    filler = pcbnew.ZONE_FILLER(board)
    zones = list(board.Zones())
    filler.Fill(board.Zones())
    board.Save(BOARD)
    print("filled", len(zones), "zones")


if __name__ == "__main__":
    main()
