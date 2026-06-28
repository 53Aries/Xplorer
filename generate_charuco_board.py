#!/usr/bin/env python3
"""
Generate a 400×400 mm ChArUco calibration board for SkewCamera.

Run this on the Pi (OpenCV is already required by SkewCamera):
  python3 generate_charuco_board.py

Output:
  charuco_400x400.png  — 300 DPI, print at exactly 100% scale
  charuco_400x400.pdf  — exact physical size PDF (if reportlab installed)

PRINT INSTRUCTIONS:
  Print at 100% / "actual size" — never "fit to page" or "scale to fit".
  Verify by measuring a square after printing: should be exactly 20 mm.
  Lay flat on bed and tape all four corners.

SKEWCAMERA CONFIG:
  Tell SkewCamera to use these board parameters (see its YAML config or
  command-line flags — exact flag names depend on SkewCamera version):
    squaresX:     20
    squaresY:     20
    squareLength: 20.0   # mm
    markerLength: 15.0   # mm
    dictionary:   DICT_5X5_250

Dependencies:
  pip3 install opencv-contrib-python numpy Pillow
  pip3 install reportlab   # optional, for PDF output
"""

import sys
import io
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ── Board parameters ──────────────────────────────────────────────────────────
SQUARES_X    = 20                        # columns
SQUARES_Y    = 20                        # rows
SQUARE_MM    = 20.0                      # physical size of each square (mm)
MARKER_MM    = 15.0                      # ArUco marker size — 75% of square
DICT_ID      = cv2.aruco.DICT_5X5_250

BOARD_MM     = SQUARES_X * SQUARE_MM    # 400 mm
assert BOARD_MM == 400.0, "Board must be 400 mm"

# ── Raster parameters ─────────────────────────────────────────────────────────
DPI          = 300
MM_PER_INCH  = 25.4
PX_PER_MM    = DPI / MM_PER_INCH        # 11.811 px/mm at 300 DPI

board_px     = int(round(BOARD_MM  * PX_PER_MM))   # 4724 px
square_px    = int(round(SQUARE_MM * PX_PER_MM))   #  236 px
marker_px    = int(round(MARKER_MM * PX_PER_MM))   #  177 px

# Label strip appended below the board (holds print instructions)
LABEL_MM     = 15.0
label_px     = int(round(LABEL_MM * PX_PER_MM))

print(f"Board:  {board_px}×{board_px} px  ({BOARD_MM:.0f}×{BOARD_MM:.0f} mm)  at {DPI} DPI")
print(f"Square: {square_px} px  ({SQUARE_MM:.0f} mm)  |  Marker: {marker_px} px  ({MARKER_MM:.0f} mm)")

# ── Generate ChArUco board ────────────────────────────────────────────────────
dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y),
    SQUARE_MM,
    MARKER_MM,
    dictionary,
)
board_img = board.generateImage((board_px, board_px), marginSize=0, borderBits=1)

# ── Add label strip ───────────────────────────────────────────────────────────
total_px  = board_px + label_px
full_img  = np.full((total_px, board_px), 255, dtype=np.uint8)
full_img[:board_px, :] = board_img

pil = Image.fromarray(full_img)
draw = ImageDraw.Draw(pil)

# 100 mm reference line in the label strip — user measures this after printing
line_start_px = int(round(50 * PX_PER_MM))
line_end_px   = int(round(150 * PX_PER_MM))
line_y        = board_px + label_px // 2
draw.line(
    [(line_start_px, line_y), (line_end_px, line_y)],
    fill=0,
    width=max(2, int(round(0.5 * PX_PER_MM))),
)
# Tick marks at each end of the reference line
tick_h = label_px // 3
for x in (line_start_px, line_end_px):
    draw.line([(x, line_y - tick_h), (x, line_y + tick_h)], fill=0,
              width=max(2, int(round(0.5 * PX_PER_MM))))

# Instruction text
try:
    font_size = max(20, int(round(3.5 * PX_PER_MM)))
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", font_size)
except OSError:
    font = ImageFont.load_default()

text = "Print at 100% — no scaling.  Verify: |←——— 100 mm ———→|"
draw.text(
    (line_end_px + int(round(5 * PX_PER_MM)), line_y - font_size // 2),
    text,
    fill=0,
    font=font,
)

full_arr = np.array(pil)

# ── PNG output ────────────────────────────────────────────────────────────────
png_path = "charuco_400x400.png"
pil.save(png_path, dpi=(DPI, DPI))
print(f"Saved:  {png_path}  ({board_px}×{total_px} px, {DPI} DPI)")

# ── PDF output (optional) ─────────────────────────────────────────────────────
pdf_path = "charuco_400x400.pdf"
try:
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas as rl_canvas

    total_mm = BOARD_MM + LABEL_MM
    c = rl_canvas.Canvas(pdf_path, pagesize=(BOARD_MM * mm, total_mm * mm))
    c.drawImage(png_path, 0, 0, width=BOARD_MM * mm, height=total_mm * mm)
    c.save()
    print(f"Saved:  {pdf_path}  ({BOARD_MM:.0f}×{total_mm:.0f} mm PDF)")
except ImportError:
    print("reportlab not found — PDF skipped (PNG is sufficient).")
    print("  pip3 install reportlab")

# ── Summary ───────────────────────────────────────────────────────────────────
print()
print("SkewCamera board parameters (set in SkewCamera YAML config or flags):")
print(f"  squaresX:     {SQUARES_X}")
print(f"  squaresY:     {SQUARES_Y}")
print(f"  squareLength: {SQUARE_MM}    # mm")
print(f"  markerLength: {MARKER_MM}    # mm")
print(f"  dictionary:   DICT_5X5_250")
