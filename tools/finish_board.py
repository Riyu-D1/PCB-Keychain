#!/usr/bin/env python3
"""Post-process keychain.kicad_pcb with pcbnew (run under KiCad's python).

- Flips PLACEMENT['B'] footprints onto the back (canonical B.* layers,
  mirrored geometry, correct silk/courtyard sides).
- Sizes reference texts to 0.9 mm and moves them to REFOFF board-space
  offsets; hides refs listed in HIDE_REFS.

Usage:
  KICAD_PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
  $KICAD_PY tools/finish_board.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcbnew
import place_board as P

MM = 1e6


def main():
    board = pcbnew.LoadBoard(P.PCB)
    fps = {f.GetReferenceAsString(): f for f in board.GetFootprints()}

    for ref, (x, y, rot, side) in P.PLACEMENT.items():
        if side != "B":
            continue
        fp = fps[ref]
        fp.Flip(fp.GetPosition(), False)

    for ref, fp in fps.items():
        t = fp.Reference()
        t.SetTextSize(pcbnew.VECTOR2I(int(0.9 * MM), int(0.9 * MM)))
        t.SetTextThickness(int(0.15 * MM))
        if ref in getattr(P, "HIDE_REFS", ()):
            t.SetVisible(False)
        elif ref in P.REFOFF:
            x, y, _r, _s = P.PLACEMENT[ref]
            dx, dy = P.REFOFF[ref]
            t.SetPosition(pcbnew.VECTOR2I(int((x + dx) * MM),
                                          int((y + dy) * MM)))

    pcbnew.SaveBoard(P.PCB, board)
    print("finish_board: flipped",
          sum(1 for p in P.PLACEMENT.values() if p[3] == "B"),
          "footprints, refs adjusted")


if __name__ == "__main__":
    main()
