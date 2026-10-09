#!/usr/bin/env python3
"""
Generator for the PCB keychain KiCad project.

Emits:
  kicad/keychain.kicad_pro
  kicad/keychain.kicad_sch
  kicad/keychain.kicad_pcb

Stock KiCad symbols/footprints are embedded verbatim so the project opens
standalone.  Design data (netlist, placements, routes) is in the tables below.
"""

import math
import os
import re
import sys
import uuid

KSHARE = os.environ.get(
    "KICAD_SHARE",
    "/Applications/KiCad/KiCad.app/Contents/SharedSupport",
)
SYMDIR = os.path.join(KSHARE, "symbols")
FPDIR = os.path.join(KSHARE, "footprints")
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUTDIR = os.path.join(ROOT, "kicad")
PROJ = "keychain"
SHEET_UUID = "9f6d5e12-7b33-4c1d-9a01-0000000000aa"
NS = uuid.UUID("9f6d5e12-7b33-4c1d-9a01-000000000001")


def U(tag):
    return str(uuid.uuid5(NS, tag))


# ---------------------------------------------------------------------------
# stock symbol extraction
# ---------------------------------------------------------------------------

_sym_cache = {}


def _load_lib(fname):
    path = os.path.join(SYMDIR, fname)
    with open(path) as f:
        return path, f.read()


def _find_symfile(name):
    if name in _sym_cache:
        return _sym_cache[name]
    lib, sym = name.split(":", 1)
    path, data = _load_lib(lib + ".kicad_sym")
    _sym_cache[name] = (path, data, sym)
    return _sym_cache[name]


def _block(data, start):
    depth = 0
    j = start
    while j < len(data):
        c = data[j]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return data[start:j + 1]
        j += 1
    raise ValueError("unbalanced parens")


def _top_kids(blk):
    """List of direct-child s-expr blocks inside blk."""
    kids = []
    i = depth = 0
    while i < len(blk):
        c = blk[i]
        if c == "(":
            if depth == 1:
                kid = _block(blk, i)
                kids.append(kid)
                i += len(kid)
                continue
            depth += 1
        elif c == ")":
            depth -= 1
        i += 1
    return kids


def _flatten(child, parent, cname, pname):
    """Resolve `extends`: parent block renamed to child, child property
    overrides applied. Sub-symbol names keep the child base name."""
    base = parent.replace(f'(symbol "{pname}"', f'(symbol "{cname}"', 1)
    base = re.sub(rf'\(symbol "{re.escape(pname)}(_\d+_\d+)"',
                  rf'(symbol "{cname}\1"', base)
    for kid in _top_kids(child):
        m = re.match(r'\(property "([^"]+)"', kid)
        if not m:
            continue
        pname_ = m.group(1)
        for bkid in _top_kids(base):
            bm = re.match(r'\(property "([^"]+)"', bkid)
            if bm and bm.group(1) == pname_:
                base = base.replace(bkid, kid, 1)
                break
        else:
            base = base.rstrip()[:-1] + "\n\t" + kid + "\n)"
    return base


def extract_symbol(qualname):
    """Return {qualified_name: block}; `extends` chains are flattened."""
    lib, name = qualname.split(":", 1)
    chain = []
    n = name
    seen = set()
    while n and n not in seen:
        seen.add(n)
        _, data, _ = _find_symfile(f"{lib}:{n}")
        i = data.find(f'(symbol "{n}"')
        blk = _block(data, i)
        chain.append((n, blk))
        m = re.search(r'\(extends "([^"]+)"', blk)
        n = m.group(1) if m else None
    blk = chain[-1][1]
    for n, child in reversed(chain[:-1]):
        blk = _flatten(child, blk, n, chain[chain.index((n, child)) + 1][0])
    blk = blk.replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)
    return {name: blk}


def symbol_pins(qualname):
    """pin number -> (x, y, ang, len); follows `extends` to base symbol."""
    lib, name = qualname.split(":", 1)
    _, data, _ = _find_symfile(qualname)
    i = data.find(f'(symbol "{name}"')
    blk = _block(data, i)
    m = re.search(r'\(extends "([^"]+)"', blk)
    if m:
        return symbol_pins(f"{lib}:{m.group(1)}")
    pins = {}
    for pm in re.finditer(r'\(pin ', blk):
        pb = _block(blk, pm.start())
        at = re.search(r'\(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\)', pb)
        ln = re.search(r'\(length ([-\d.]+)\)', pb)
        num = re.search(r'\(number "([^"]+)"', pb)
        if at and ln and num:
            pins[num.group(1)] = (float(at.group(1)), float(at.group(2)),
                                  float(at.group(3)), float(ln.group(1)))
    return pins


# ---------------------------------------------------------------------------
# netlist
# ---------------------------------------------------------------------------

NETS = {}


def N(name, *pins):
    NETS.setdefault(name, []).extend(pins)


# --- usb-c input -----------------------------------------------------------
N("VBUS", ("J3", "A4"), ("J3", "B4"), ("J3", "A9"), ("J3", "B9"),
   ("D1", "2"), ("C1", "1"), ("F1", "1"))
N("VBUS_F", ("F1", "2"), ("U2", "4"), ("LED1", "2"))
N("CC1", ("J3", "A5"), ("R1", "1"), ("U6", "4"))
N("CC2", ("J3", "B5"), ("R2", "1"), ("U6", "5"))
N("USB_DP_CONN", ("J3", "A6"), ("J3", "B6"), ("R3", "1"), ("U6", "1"))
N("USB_DM_CONN", ("J3", "A7"), ("J3", "B7"), ("R4", "1"), ("U6", "3"))
N("USB_DP", ("R3", "2"), ("U1", "18"))
N("USB_DM", ("R4", "2"), ("U1", "17"))
# --- charger ---------------------------------------------------------------
N("VBAT", ("U2", "3"), ("J2", "1"), ("C2", "1"), ("R9", "1"), ("SW3", "1"))
N("CHG_STAT", ("U2", "1"), ("R6", "2"), ("U1", "19"))
N("CHG_LED", ("LED1", "1"), ("R6", "1"))
N("PROG", ("U2", "5"), ("R5", "1"))
# --- switched rail / 3v3 ---------------------------------------------------
N("VBAT_SYS", ("SW3", "2"), ("U3", "1"), ("U3", "3"), ("C3", "1"),
   ("U4", "6"), ("L1", "1"), ("C8", "1"))
N("+3V3", ("U3", "5"), ("C4", "1"), ("C5", "1"), ("C14", "1"), ("U1", "3"),
   ("J1", "2"), ("R7", "1"), ("R8", "1"), ("LED2", "2"))
# --- boost + led driver ----------------------------------------------------
N("+5V", ("U4", "5"), ("C9", "1"), ("U5", "5"),
   ("LED3", "4"), ("LED4", "4"), ("LED5", "4"), ("LED6", "4"),
   ("C10", "1"), ("C11", "1"), ("C12", "1"), ("C13", "1"),
   ("R11", "1"))
N("BOOST_SW", ("U4", "1"), ("L1", "2"))
N("LED_FB", ("U4", "3"), ("R11", "2"), ("R12", "1"))
N("BOOST_EN", ("U4", "4"), ("R13", "1"), ("U1", "25"))
N("LED_DAT_3V", ("U1", "24"), ("U5", "2"))
N("LED_DIN", ("U5", "4"), ("R14", "1"))
N("LED_D0", ("R14", "2"), ("LED3", "3"))
N("LED_C1", ("LED3", "1"), ("LED4", "3"))
N("LED_C2", ("LED4", "1"), ("LED5", "3"))
N("LED_C3", ("LED5", "1"), ("LED6", "3"))
# --- mcu io ----------------------------------------------------------------
N("USER1", ("U1", "12"), ("SW1", "1"))
N("USER2", ("U1", "13"), ("SW2", "1"))
N("BOOT", ("U1", "23"), ("R8", "2"), ("SW4", "1"))
N("EN", ("U1", "8"), ("R7", "2"), ("C6", "1"), ("SW5", "1"))
N("BAT_ADC", ("U1", "5"), ("R9", "2"), ("R10", "1"), ("C7", "1"))
N("PWR_LED", ("LED2", "1"), ("R15", "1"))
# --- oled header (module pinout: GND VCC SCK SDA RES DC CS) -----------------
N("OLED_SCK", ("U1", "15"), ("J1", "3"))
N("OLED_MOSI", ("U1", "16"), ("J1", "4"))
N("OLED_RES", ("U1", "6"), ("J1", "5"))
N("OLED_DC", ("U1", "9"), ("J1", "6"))
N("OLED_CS", ("U1", "10"), ("J1", "7"))
# --- ground ----------------------------------------------------------------
N("GND",
  ("J3", "A1"), ("J3", "B1"), ("J3", "A12"), ("J3", "B12"), ("J3", "SH"),
  ("D1", "1"), ("C1", "2"), ("R1", "2"), ("R2", "2"), ("U6", "2"),
  ("U2", "2"), ("C2", "2"), ("J2", "2"), ("R5", "2"), ("R6", "2"),
  ("C3", "2"), ("C4", "2"), ("C5", "2"), ("C6", "2"), ("C7", "2"),
  ("C14", "2"), ("U3", "2"),
  ("R10", "2"),
  ("U4", "2"), ("C8", "2"), ("C9", "2"), ("R12", "2"), ("R13", "2"),
  ("U5", "1"), ("U5", "3"), ("R15", "2"),
  ("LED3", "2"), ("LED4", "2"), ("LED5", "2"), ("LED6", "2"),
  ("C10", "2"), ("C11", "2"), ("C12", "2"), ("C13", "2"),
  ("SW1", "2"), ("SW2", "2"), ("SW4", "2"), ("SW5", "2"),
  ("U1", "1"), ("U1", "2"), ("U1", "11"), ("U1", "14"),
  ("U1", "36"), ("U1", "37"), ("U1", "38"), ("U1", "39"), ("U1", "40"),
  ("U1", "41"), ("U1", "42"), ("U1", "43"), ("U1", "44"), ("U1", "45"),
  ("U1", "46"), ("U1", "47"), ("U1", "48"), ("U1", "49"), ("U1", "50"),
  ("U1", "51"), ("U1", "52"), ("U1", "53"),
  ("J1", "1"),
  )

for k in list(NETS):
    seen = []
    for p in NETS[k]:
        if p not in seen:
            seen.append(p)
    NETS[k] = seen

NC_PINS = [
    ("J3", "A8"), ("J3", "B8"), ("J3", "SH2"),
    ("U1", "4"), ("U1", "7"), ("U1", "20"), ("U1", "21"), ("U1", "22"),
    ("U1", "26"), ("U1", "27"), ("U1", "28"), ("U1", "29"), ("U1", "30"),
    ("U1", "31"), ("U1", "32"), ("U1", "33"), ("U1", "34"), ("U1", "35"),
    ("U6", "6"), ("U3", "4"), ("SW3", "3"), ("LED6", "1"),
]

# ---------------------------------------------------------------------------
# schematic components: (ref, libsym, value, footprint, x, y, rot)
# ---------------------------------------------------------------------------
COMPONENTS = [
    # --- usb-c input (left column) ----------------------------------------
    ("J3",  "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C",
     "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", 55, 60, 0),
    ("U6",  "Power_Protection:ESDA6V1-5SC6", "ESDA6V1-5SC6",
     "Package_TO_SOT_SMD:SOT-23-6", 90, 55, 0),
    ("R3",  "Device:R", "22",
     "Resistor_SMD:R_0603_1608Metric", 108, 45, 0),
    ("R4",  "Device:R", "22",
     "Resistor_SMD:R_0603_1608Metric", 108, 55, 0),
    ("R1",  "Device:R", "5.1k",
     "Resistor_SMD:R_0603_1608Metric", 75, 95, 0),
    ("R2",  "Device:R", "5.1k",
     "Resistor_SMD:R_0603_1608Metric", 83, 95, 0),
    ("D1",  "Diode:SMAJ5.0A", "SMAJ5.0A",
     "Diode_SMD:D_SMA", 75, 110, 0),
    ("C1",  "Device:C", "10uF",
     "Capacitor_SMD:C_0603_1608Metric", 85, 110, 0),
    ("F1",  "Device:Polyfuse", "0.5A",
     "Resistor_SMD:R_1206_3216Metric", 100, 85, 0),
    ("LED1", "Device:LED", "RED",
     "LED_SMD:LED_0603_1608Metric", 115, 100, 0),
    ("R6",  "Device:R", "1k",
     "Resistor_SMD:R_0603_1608Metric", 125, 100, 0),
    # --- charger -----------------------------------------------------------
    ("U2",  "Battery_Management:MCP73831-2-OT", "MCP73831",
     "Package_TO_SOT_SMD:SOT-23-5", 150, 55, 0),
    ("R5",  "Device:R", "4.7k",
     "Resistor_SMD:R_0603_1608Metric", 132, 70, 0),
    ("C2",  "Device:C", "4.7uF",
     "Capacitor_SMD:C_0603_1608Metric", 168, 70, 0),
    ("J2",  "Connector_Generic:Conn_01x02", "LiPo 3.7V",
     "Connector_JST:JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal", 180, 55, 0),
    # --- power switch + ldo -------------------------------------------------
    ("SW3", "Switch:SW_SPDT", "PWR",
     "Button_Switch_SMD:SW_SPDT_Shouhan_MSK12C02", 205, 55, 0),
    ("U3",  "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3",
     "Package_TO_SOT_SMD:SOT-23-5", 228, 60, 0),
    ("C3",  "Device:C", "1uF",
     "Capacitor_SMD:C_0603_1608Metric", 214, 78, 0),
    ("C4",  "Device:C", "10uF",
     "Capacitor_SMD:C_0603_1608Metric", 240, 78, 0),
    # --- boost + led driver -------------------------------------------------
    ("L1",  "Device:L", "2.2uH",
     "Inductor_SMD:L_1008_2520Metric", 268, 60, 0),
    ("U4",  "Regulator_Switching:TLV61046ADB", "TLV61046A",
     "Package_TO_SOT_SMD:SOT-23-6", 288, 60, 0),
    ("C8",  "Device:C", "10uF",
     "Capacitor_SMD:C_0603_1608Metric", 268, 80, 0),
    ("C9",  "Device:C", "10uF",
     "Capacitor_SMD:C_0603_1608Metric", 288, 80, 0),
    ("R11", "Device:R", "510k",
     "Resistor_SMD:R_0603_1608Metric", 302, 55, 0),
    ("R12", "Device:R", "97.6k",
     "Resistor_SMD:R_0603_1608Metric", 302, 70, 0),
    ("R13", "Device:R", "100k",
     "Resistor_SMD:R_0603_1608Metric", 288, 92, 0),
    ("U5",  "74xGxx:74AHCT1G125", "74AHCT1G125",
     "Package_TO_SOT_SMD:SOT-23-5", 320, 60, 0),
    ("R14", "Device:R", "330",
     "Resistor_SMD:R_0603_1608Metric", 334, 60, 0),
    # --- rgb leds -----------------------------------------------------------
    ("LED3", "LED:SK6812MINI", "SK6812MINI",
     "LED_SMD:LED_SK6812MINI_PLCC4_3.5x3.5mm_P1.75mm", 365, 40, 0),
    ("LED4", "LED:SK6812MINI", "SK6812MINI",
     "LED_SMD:LED_SK6812MINI_PLCC4_3.5x3.5mm_P1.75mm", 365, 56, 0),
    ("LED5", "LED:SK6812MINI", "SK6812MINI",
     "LED_SMD:LED_SK6812MINI_PLCC4_3.5x3.5mm_P1.75mm", 365, 76, 0),
    ("LED6", "LED:SK6812MINI", "SK6812MINI",
     "LED_SMD:LED_SK6812MINI_PLCC4_3.5x3.5mm_P1.75mm", 365, 96, 0),
    ("C10", "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 382, 40, 0),
    ("C11", "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 382, 56, 0),
    ("C12", "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 382, 76, 0),
    ("C13", "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 382, 96, 0),
    # --- mcu ----------------------------------------------------------------
    ("U1",  "RF_Module:ESP32-C6-MINI-1", "ESP32-C6-MINI-1",
     "RF_Module:ESP32-C6-MINI-1", 160, 160, 0),
    ("C5",  "Device:C", "10uF",
     "Capacitor_SMD:C_0603_1608Metric", 160, 120, 0),
    ("R7",  "Device:R", "10k",
     "Resistor_SMD:R_0603_1608Metric", 125, 200, 0),
    ("C6",  "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 137, 200, 0),
    ("R8",  "Device:R", "10k",
     "Resistor_SMD:R_0603_1608Metric", 152, 200, 0),
    ("SW4", "Switch:SW_Push", "BOOT",
     "Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A", 175, 200, 0),
    ("SW5", "Switch:SW_Push", "RST",
     "Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A", 190, 200, 0),
    ("SW1", "Switch:SW_Push", "USER1",
     "Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A", 210, 200, 0),
    ("SW2", "Switch:SW_Push", "USER2",
     "Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A", 225, 200, 0),
    ("R9",  "Device:R", "100k",
     "Resistor_SMD:R_0603_1608Metric", 245, 165, 0),
    ("R10", "Device:R", "100k",
     "Resistor_SMD:R_0603_1608Metric", 255, 180, 0),
    ("C7",  "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 245, 195, 0),
    ("LED2", "Device:LED", "GRN",
     "LED_SMD:LED_0603_1608Metric", 268, 165, 0),
    ("R15", "Device:R", "1k",
     "Resistor_SMD:R_0603_1608Metric", 268, 178, 0),
    # --- oled header ---------------------------------------------------------
    ("J1",  "Connector_Generic:Conn_01x07", "OLED 1.5in SSD1327",
     "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical",
     300, 160, 0),
    ("C14", "Device:C", "100nF",
     "Capacitor_SMD:C_0603_1608Metric", 300, 185, 0),
    # --- keyring hole --------------------------------------------------------
    ("H1",  "Device:MountingHole_Pad", "Keyring",
     "MountingHole:MountingHole_3.2mm_M3_Pad", 365, 130, 0),
]

# PWR_FLAG placements: (net, attach_pin) -> flag stacked at that pin's tip
# (only on nets with no power-output driver: +3V3/+5V/VBAT are driven by ICs)
PWR_FLAGS = [("VBUS", ("F1", "1")), ("VBUS_F", ("F1", "2")),
             ("VBAT_SYS", ("SW3", "2")), ("BOOST_SW", ("L1", "2")),
             ("GND", ("J3", "A1"))]

SECTION_TEXT = [
    ("USB-C INPUT + PROTECTION", 40, 30),
    ("LiPo CHARGER MCP73831 (I=~215mA)", 130, 40),
    ("PWR SWITCH + 3V3 LDO", 200, 42),
    ("5V BOOST + LEVEL SHIFT", 265, 42),
    ("ADDRESSABLE RGB LEDs (x4)", 355, 28),
    ("MCU ESP32-C6-MINI-1 (BLE/WiFi6/USB)", 130, 125),
    ("OLED 1.5in SSD1327 128x128 SPI", 285, 125),
    ("KEYRING HOLE (GND pad)", 355, 118),
]

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def rot_pt(x, y, deg):
    r = math.radians(deg)
    return (x * math.cos(r) + y * math.sin(r),
            -x * math.sin(r) + y * math.cos(r))


def pin_tip(sx, sy, srot, poff):
    px, py, ang, ln = poff
    ox, oy = rot_pt(px, -py, srot)
    return sx + ox, sy + oy, (ang - srot) % 360


def outward_vec(srot, poff):
    px, py, ang, ln = poff
    gx, gy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    return rot_pt(-gx, gy, srot)


def dirv(ang):
    r = math.radians(ang)
    return math.cos(r), math.sin(r)


def fmt(x):
    s = f"{x:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def esc(s):
    return s.replace('"', '\\"')


def prop(name, val, x, y, rot=0, hide=False):
    h = "\n\t\t\t\t\t(hide yes)" if hide else ""
    return (f'\t\t(property "{name}" "{esc(val)}"\n'
            f'\t\t\t(at {fmt(x)} {fmt(y)} {rot})\n'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t'
            f'(size 1.27 1.27)\n\t\t\t\t){h}\n\t\t\t)\n\t\t)')


# ---------------------------------------------------------------------------
def build_pro():
    return '''{
  "board": {
    "design_settings": {
      "defaults": {
        "board_outline_line_width": 0.1, "copper_line_width": 0.2,
        "copper_text_size_h": 1.5, "copper_text_size_v": 1.5,
        "copper_text_thickness": 0.3, "other_line_width": 0.15,
        "silk_line_width": 0.15, "silk_text_size_h": 1.0,
        "silk_text_size_v": 1.0, "silk_text_thickness": 0.15
      },
      "diff_pair_dimensions": [], "drc_exclusions": [],
      "rules": {
        "max_error": 0.005, "min_clearance": 0.15,
        "min_copper_edge_clearance": 0.3, "min_hole_clearance": 0.25,
        "min_hole_to_hole": 0.25, "min_silk_clearance": 0.0,
        "min_track_width": 0.15, "min_via_annulus": 0.15,
        "min_via_diameter": 0.5, "solder_mask_to_copper_clearance": 0.0,
        "use_height_for_length_calcs": true
      },
      "track_widths": [0.25, 0.4, 0.5, 0.8],
      "via_dimensions": [{"diameter": 0.8, "drill": 0.4}]
    },
    "ipc2581": {"dist": "", "distpn": "", "internal_id": "", "mfg": "", "rev": ""},
    "layer_presets": [], "viewports": []
  },
  "boards": [], "cvpcb": {"equivalence_files": []},
  "erc": {"erc_exclusions": [], "meta": {"version": 0}, "pin_map": [],
          "rule_severities": {}},
  "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
  "meta": {"filename": "%s.kicad_pro", "version": 1},
  "net_settings": {
    "classes": [{
      "bus_width": 12, "clearance": 0.15, "diff_pair_gap": 0.25,
      "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "line_style": 0,
      "microvia_diameter": 0.3, "microvia_drill": 0.1, "name": "Default",
      "pcb_color": "rgba(0,0,0,0)", "priority": 0,
      "schematic_color": "rgba(0,0,0,0)", "track_width": 0.25,
      "via_diameter": 0.8, "via_drill": 0.4, "wire_width": 6}],
    "meta": {"version": 3}, "net_colors": null,
    "netclass_assignments": null, "netclass_patterns": []
  },
  "pcbnew": {"last_paths": {"gencad": "", "idf": "", "netlist": "",
             "specctra_dsn": "", "step": "", "vrml": ""},
             "page_layout_descr_file": ""},
  "schematic": {"annotate_start_num": 0, "bom_fmt_presets": [],
    "bom_fmt_specs": [], "bom_presets": [], "bom_specs": [],
    "connection_grid_size": 50.0, "datasheet_urls": "",
    "drawing": {"default_line_thickness": 6.0, "default_text_size": 50.0,
      "field_names": [], "intersheets_ref_own_page": false,
      "intersheets_ref_prefix": "", "intersheets_ref_short": false,
      "intersheets_ref_show": false, "intersheets_ref_suffix": "",
      "junction_size_choice": 3, "label_size_choice": 3,
      "no_connect_size_choice": 0, "pin_symbol_scale": 1.0},
    "legacy_lib_dir": "", "legacy_lib_list": [], "meta": {"version": 1},
    "page_layout_descr_file": "", "plot_directory": "",
    "spice_adjust_passive_values": false,
    "spice_current_sheet_as_root": false,
    "spice_external_command": "spice \\"%%I\\"",
    "spice_model_current_sheet_as_root": true,
    "spice_save_all_currents": false, "spice_save_all_dissipations": false,
    "spice_save_all_voltages": false, "subpart_first_id": 65,
    "subpart_id_separator": 0},
  "sheets": [["%s", ""]],
  "text_variables": {}
}
''' % (PROJ, SHEET_UUID)


def build_sch():
    used = {}
    for ref, libsym, *_ in COMPONENTS:
        for k, blk in extract_symbol(libsym).items():
            used[f"{libsym.split(':')[0]}:{k}"] = blk
    for k, blk in extract_symbol("power:PWR_FLAG").items():
        used["power:" + k] = blk
    libsec = "\n".join("\t" + b.replace("\n", "\n\t")
                       for b in used.values())

    out = ['(kicad_sch', '\t(version 20250114)', '\t(generator "eeschema")',
           '\t(generator_version "10.0")',
           f'\t(uuid "{U("schematic")}")', '\t(paper "A3")',
           '\t(title_block',
           '\t\t(title "PCB Keychain - BLE OLED badge")',
           '\t\t(rev "A")',
           '\t\t(comment 1 "ESP32-C6 + 1.5in SSD1327 OLED + SK6812 x4 + LiPo")',
           '\t)', '\t(lib_symbols', libsec, '\t)']

    pinnet = {}
    for net, pins in NETS.items():
        for rp in pins:
            pinnet[rp] = net

    labels = []
    ncmarks = []
    psyms = []
    tip_of = {}   # (ref,pin) -> (x,y,outward)

    for ref, libsym, value, fp, sx, sy, srot in COMPONENTS:
        sx = round(sx / 1.27) * 1.27
        sy = round(sy / 1.27) * 1.27
        pins = symbol_pins(libsym)
        for net, plist in NETS.items():
            for rp in plist:
                if rp[0] == ref:
                    assert rp[1] in pins, f"{ref} pin {rp[1]} not in {libsym}"
        L = ['\t(symbol', f'\t\t(lib_id "{libsym}")',
             f'\t\t(at {fmt(sx)} {fmt(sy)} {srot})', '\t\t(unit 1)',
             '\t\t(exclude_from_sim no)', '\t\t(in_bom yes)',
             '\t\t(on_board yes)', '\t\t(dnp no)',
             f'\t\t(uuid "{U("sym:"+ref)}")',
             prop("Reference", ref, sx, sy - 6),
             prop("Value", value, sx, sy + 6),
             prop("Footprint", fp, sx, sy, hide=True),
             prop("Datasheet", "", sx, sy, hide=True)]
        for num, poff in pins.items():
            L.append(f'\t\t(pin "{num}"')
            L.append(f'\t\t\t(uuid "{U("pin:"+ref+":"+num)}")')
            L.append('\t\t)')
            tx, ty, tang = pin_tip(sx, sy, srot, poff)
            dx, dy = outward_vec(srot, poff)
            tip_of[(ref, num)] = (tx, ty, dx, dy)
            key = (ref, num)
            if key in NC_PINS:
                ncmarks.append((tx, ty))
                continue
            net = pinnet.get(key)
            if net is None:
                ncmarks.append((tx, ty))
                continue
            labels.append((net, tx, ty, dx, dy))
        L += ['\t\t(instances', f'\t\t\t(project "{PROJ}"',
              f'\t\t\t\t(path "/{SHEET_UUID}"',
              f'\t\t\t\t\t(reference "{ref}")', '\t\t\t\t\t(unit 1)',
              '\t\t\t\t)', '\t\t\t)', '\t\t)', '\t)']
        out.append("\n".join(L))

    # guard: no two different-net pins may share a coordinate
    tip_net = {}
    for key, (tx, ty, dx, dy) in tip_of.items():
        net = pinnet.get(key)
        if net is None or key in NC_PINS:
            continue
        tag = (fmt(tx), fmt(ty))
        if tag in tip_net and tip_net[tag] != (net, None):
            prev_net, prev_pin = tip_net[tag]
            assert prev_net == net, \
                f"pin collision at {tag}: {prev_pin}({prev_net}) vs {key}({net})"
        tip_net[tag] = (net, key)

    # PWR_FLAG symbols stacked at chosen pin tips
    for net, atpin in PWR_FLAGS:
        tx, ty, dx, dy = tip_of[atpin]
        psyms.append(("power:PWR_FLAG", tx, ty, dx, dy, net))

    seen = set()
    for net, ex, ey, dx, dy in labels:
        rot = 180 if dx < 0 else 0
        jus = "right bottom" if rot == 180 else (
            "left top" if dy > 0 else "left bottom")
        tag = (net, fmt(ex), fmt(ey))
        if tag in seen:
            continue
        seen.add(tag)
        out.append(f'\t(label "{esc(net)}" (at {fmt(ex)} {fmt(ey)} {rot})'
                   f' (effects (font (size 1.27 1.27)) (justify {jus}))'
                   f' (uuid "{U("l:"+str(tag))}"))')

    pseen = set()
    pwr_flag_no = 0
    for psym, ex, ey, dx, dy, net in psyms:
        tag = (psym, fmt(ex), fmt(ey))
        if tag in pseen:
            continue
        pseen.add(tag)
        pwr_flag_no += 1
        pang = symbol_pins(psym)["1"][2]
        ga = math.degrees(math.atan2(-math.sin(math.radians(pang)),
                                     math.cos(math.radians(pang))))
        wa = math.degrees(math.atan2(dy, dx))
        prot = round((ga - wa) / 90) * 90 % 360
        out += ['\t(symbol', f'\t\t(lib_id "{psym}")',
                f'\t\t(at {fmt(ex)} {fmt(ey)} {prot})', '\t\t(unit 1)',
                '\t\t(exclude_from_sim no)', '\t\t(in_bom no)',
                '\t\t(on_board no)', '\t\t(dnp no)',
                f'\t\t(uuid "{U("ps:"+str(ex)+str(ey)+psym)}")',
                prop("Reference", "#PWR%02d" % pwr_flag_no, ex, ey + 3.81,
                     hide=True),
                prop("Value", net if not psym.endswith("FLAG") else "PWR_FLAG",
                     ex, ey - 3.81, hide=True),
                prop("Footprint", "", ex, ey, hide=True),
                prop("Datasheet", "", ex, ey, hide=True),
                '\t\t(pin "1"',
                f'\t\t\t(uuid "{U("pp:"+str(ex)+str(ey)+psym)}")', '\t\t)',
                '\t\t(instances', f'\t\t\t(project "{PROJ}"',
                f'\t\t\t\t(path "/{SHEET_UUID}"',
                f'\t\t\t\t\t(reference "#PWR%02d")' % pwr_flag_no,
                '\t\t\t\t\t(unit 1)', '\t\t\t\t)', '\t\t\t)', '\t\t)', '\t)']

    for x, y in ncmarks:
        out.append(f'\t(no_connect (at {fmt(x)} {fmt(y)}) '
                   f'(uuid "{U("nc:"+fmt(x)+":"+fmt(y))}"))')

    for txt, x, y in SECTION_TEXT:
        out.append(f'\t(text "{txt}" (at {x} {y} 0) (effects '
                   '(font (size 2 2) (bold yes)) (justify left))'
                   f' (uuid "{U("t:"+txt)}"))')

    out += ['\t(sheet_instances', '\t\t(path "/"', '\t\t\t(page "1")',
            '\t\t)', '\t)', '\t(embedded_fonts no)', ')']
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------------------
# pcb
# ---------------------------------------------------------------------------

# board geometry (mm, absolute coords; y down)
BX0, BY0 = 20.0, 25.0            # top-left corner of main rect
BW, BH = 68.0, 52.0
LUG_X0, LUG_X1, LUG_TOP, LUG_R = 26.0, 42.0, 13.0, 3.0   # lug on top edge
HOLE = (BX0 + 14.0, BY0 - 8.0)                            # = (34,17)


def load_fp(qualname):
    lib, name = qualname.split(":", 1)
    path = os.path.join(FPDIR, lib + ".pretty", name + ".kicad_mod")
    with open(path) as f:
        return f.read()


def fp_pads(qualname):
    """padname -> list of (x, y) local coords."""
    data = load_fp(qualname)
    pads = {}
    for m in re.finditer(r'\(pad "([^"]*)" \S+ \S+\s*\(at ([-\d.]+) ([-\d.]+)',
                         data):
        pads.setdefault(m.group(1), []).append(
            (float(m.group(2)), float(m.group(3))))
    return pads


PADABS = {}   # (ref, padname) -> (x, y)  absolute board coords


def xform(fx, fy, fr, side, px, py):
    """footprint-local -> absolute.  Bottom footprints are X-mirrored."""
    if side == "B":
        px = -px
    ox, oy = rot_pt(px, py, fr)
    return fx + ox, fy + oy


def build_pcb():
    return ""


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    if "sch" in sys.argv or len(sys.argv) == 1:
        open(os.path.join(OUTDIR, PROJ + ".kicad_sch"), "w").write(build_sch())
    if "pcb" in sys.argv or len(sys.argv) == 1:
        open(os.path.join(OUTDIR, PROJ + ".kicad_pcb"), "w").write(build_pcb())
    if "pro" in sys.argv or len(sys.argv) == 1:
        open(os.path.join(OUTDIR, PROJ + ".kicad_pro"), "w").write(build_pro())


if __name__ == "__main__":
    main()
