"""Generate event-driven HALO status cards for the OSD bitmap layer.

The board loads one 300x82 palette-indexed SSBMP at a time.  Index 31 is
transparent, matching the polygon OSD implementation; indices 0, 1 and 3 are
red, green and yellow in colorLUT.sscl.  Keeping text pre-rendered avoids any
font or bitmap encoding work in the per-frame detection path.
"""

from pathlib import Path
import struct


FONT = {
    "A": (".XXX.", "X...X", "X...X", "XXXXX", "X...X", "X...X", "X...X"),
    "B": ("XXXX.", "X...X", "X...X", "XXXX.", "X...X", "X...X", "XXXX."),
    "C": (".XXXX", "X....", "X....", "X....", "X....", "X....", ".XXXX"),
    "D": ("XXXX.", "X...X", "X...X", "X...X", "X...X", "X...X", "XXXX."),
    "E": ("XXXXX", "X....", "X....", "XXXX.", "X....", "X....", "XXXXX"),
    "F": ("XXXXX", "X....", "X....", "XXXX.", "X....", "X....", "X...."),
    "G": (".XXXX", "X....", "X....", "X.XXX", "X...X", "X...X", ".XXX."),
    "H": ("X...X", "X...X", "X...X", "XXXXX", "X...X", "X...X", "X...X"),
    "I": ("XXXXX", "..X..", "..X..", "..X..", "..X..", "..X..", "XXXXX"),
    "J": ("..XXX", "...X.", "...X.", "...X.", "...X.", "X..X.", ".XX.."),
    "K": ("X...X", "X..X.", "X.X..", "XX...", "X.X..", "X..X.", "X...X"),
    "L": ("X....", "X....", "X....", "X....", "X....", "X....", "XXXXX"),
    "M": ("X...X", "XX.XX", "X.X.X", "X.X.X", "X...X", "X...X", "X...X"),
    "N": ("X...X", "XX..X", "XX..X", "X.X.X", "X..XX", "X..XX", "X...X"),
    "O": (".XXX.", "X...X", "X...X", "X...X", "X...X", "X...X", ".XXX."),
    "P": ("XXXX.", "X...X", "X...X", "XXXX.", "X....", "X....", "X...."),
    "Q": (".XXX.", "X...X", "X...X", "X...X", "X.X.X", "X..X.", ".XX.X"),
    "R": ("XXXX.", "X...X", "X...X", "XXXX.", "X.X..", "X..X.", "X...X"),
    "S": (".XXXX", "X....", "X....", ".XXX.", "....X", "....X", "XXXX."),
    "T": ("XXXXX", "..X..", "..X..", "..X..", "..X..", "..X..", "..X.."),
    "U": ("X...X", "X...X", "X...X", "X...X", "X...X", "X...X", ".XXX."),
    "V": ("X...X", "X...X", "X...X", "X...X", "X...X", ".X.X.", "..X.."),
    "W": ("X...X", "X...X", "X...X", "X.X.X", "X.X.X", "XX.XX", "X...X"),
    "X": ("X...X", "X...X", ".X.X.", "..X..", ".X.X.", "X...X", "X...X"),
    "Y": ("X...X", "X...X", ".X.X.", "..X..", "..X..", "..X..", "..X.."),
    "Z": ("XXXXX", "....X", "...X.", "..X..", ".X...", "X....", "XXXXX"),
    " ": (".....", ".....", ".....", ".....", ".....", ".....", "....."),
}

WIDTH = 300
HEIGHT = 82
TRANSPARENT = 31
RED = 0
GREEN = 1
YELLOW = 3

CARDS = {
    "status_home.ssbmp": ("HOME", "FAMILY CARE", GREEN),
    "status_away.ssbmp": ("AWAY", "PERSON GUARD", YELLOW),
    "status_sleep.ssbmp": ("SLEEP", "NIGHT GUARD", GREEN),
    "status_config.ssbmp": ("CONFIG", "ALARM OFF", YELLOW),
    "status_no_zone.ssbmp": ("NO ZONE", "DRAW AREA", YELLOW),
    "status_alarm.ssbmp": ("ALARM", "ZONE ENTRY", RED),
    "status_degraded.ssbmp": ("SYSTEM", "DEGRADED", RED),
}


def draw_text(pixels, text, x, y, scale, color):
    cursor = x
    for char in text:
        glyph = FONT[char]
        for gy, row in enumerate(glyph):
            for gx, value in enumerate(row):
                if value != "X":
                    continue
                for sy in range(scale):
                    for sx in range(scale):
                        px = cursor + gx * scale + sx
                        py = y + gy * scale + sy
                        if 0 <= px < WIDTH and 0 <= py < HEIGHT:
                            pixels[py * WIDTH + px] = color
        cursor += 6 * scale


def draw_card(title, subtitle, accent):
    pixels = [TRANSPARENT] * (WIDTH * HEIGHT)
    for y in range(2, HEIGHT - 2):
        for x in range(2, WIDTH - 2):
            if x < 5 or x >= WIDTH - 5 or y < 5 or y >= HEIGHT - 5:
                pixels[y * WIDTH + x] = accent
    draw_text(pixels, title, 15, 10, 4, accent)
    draw_text(pixels, subtitle, 16, 55, 2, YELLOW if accent != YELLOW else GREEN)
    return pixels


def main():
    output_dir = Path(__file__).resolve().parent.parent / "ssne_ai_yolo_coco" / "app_assets"
    output_dir.mkdir(parents=True, exist_ok=True)
    header = b"MBSS" + struct.pack("<III", WIDTH, HEIGHT, 32)
    for filename, (title, subtitle, accent) in CARDS.items():
        pixels = draw_card(title, subtitle, accent)
        output_path = output_dir / filename
        output_path.write_bytes(header + bytes(pixels))
        print(f"Wrote {output_path} ({output_path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
