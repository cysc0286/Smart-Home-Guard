"""Generate ALERT.ssbmp (74x74 palette-indexed) for the OSD layer 2 alarm overlay.

SSBMP format (reverse engineered from SmartSens existing assets):
  Offset 0x00 : 4 bytes  magic "MBSS"
  Offset 0x04 : 4 bytes  width  (LE int32) = 74
  Offset 0x08 : 4 bytes  height (LE int32) = 74
  Offset 0x0C : 4 bytes  unknown / palette (LE int32) = 25
  Offset 0x10 : width*height bytes  -- color LUT indices

Colors used (matches colorLUT.sscl):
  index 0 = Red    (background)
  index 1 = Green  (text "ALERT")

Usage:
  python gen_alert_ssbmp.py
Outputs:
  ../ssne_ai_yolo_coco/app_assets/alert.ssbmp
"""

import os
import struct

# 5x7 bitmap font (letters used in "ALERT" + extra)
FONT = {
    'A': [
        ".XXX.",
        "X...X",
        "X...X",
        "XXXXX",
        "X...X",
        "X...X",
        "X...X",
    ],
    'L': [
        "X....",
        "X....",
        "X....",
        "X....",
        "X....",
        "X....",
        "XXXXX",
    ],
    'E': [
        "XXXXX",
        "X....",
        "X....",
        "XXXX.",
        "X....",
        "X....",
        "XXXXX",
    ],
    'R': [
        "XXXX.",
        "X...X",
        "X...X",
        "XXXX.",
        "X.X..",
        "X..X.",
        "X...X",
    ],
    'T': [
        "XXXXX",
        "..X..",
        "..X..",
        "..X..",
        "..X..",
        "..X..",
        "..X..",
    ],
    '!': [
        "..X..",
        "..X..",
        "..X..",
        "..X..",
        "..X..",
        ".....",
        "..X..",
    ],
    ' ': [
        ".....",
        ".....",
        ".....",
        ".....",
        ".....",
        ".....",
        ".....",
    ],
}

W = 74
H = 74
BG_INDEX = 0   # red background
FG_INDEX = 1   # green/foreground text
SCALE    = 2   # 2x pixel scale -> 10x14 per letter

def render_text(canvas, text, x_start, y_start, scale, fg, bg=None):
    """Render `text` onto canvas at (x_start, y_start) with integer pixel scaling."""
    cursor_x = x_start
    for ch in text:
        glyph = FONT.get(ch.upper())
        if glyph is None:
            glyph = FONT[' ']
        gh = len(glyph)
        gw = len(glyph[0])
        for gy in range(gh):
            for gx in range(gw):
                if glyph[gy][gx] == 'X':
                    for sy in range(scale):
                        for sx in range(scale):
                            px = cursor_x + gx * scale + sx
                            py = y_start  + gy * scale + sy
                            if 0 <= px < W and 0 <= py < H:
                                canvas[py * W + px] = fg
        cursor_x += (gw + 1) * scale  # one pixel gap between letters, scaled

def main():
    # 红底 (index 0)
    pixels = [BG_INDEX] * (W * H)

    # 渲染 "ALERT" 居中，2x scale -> 每字10x14, 总宽 = 5*10 + 4*2 = 58, 余量= (74-58)/2=8
    text = "ALERT"
    text_w = (5 * 5 + 4 * 1) * SCALE  # 5 letters * 5px wide + 4 gaps of 1px, scaled
    text_h = 7 * SCALE
    x0 = (W - text_w) // 2
    y0 = (H - text_h) // 2
    render_text(pixels, text, x0, y0, SCALE, FG_INDEX, BG_INDEX)

    # 顶部/底部加绿色装饰条增强报警感
    for y in (2, 3, H - 4, H - 3):
        for x in range(4, W - 4):
            pixels[y * W + x] = FG_INDEX

    out_dir = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "ssne_ai_yolo_coco", "app_assets"))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "alert.ssbmp")

    header = b"MBSS" + struct.pack("<III", W, H, 25)
    body   = bytes(pixels)
    with open(out_path, "wb") as f:
        f.write(header + body)

    expected_size = 16 + W * H
    actual_size = len(header) + len(body)
    print(f"Wrote {out_path}")
    print(f"  size={actual_size} bytes (expected {expected_size})  W={W} H={H}")
    print(f"  bg=index{BG_INDEX} (red)  fg=index{FG_INDEX} (green)")

if __name__ == "__main__":
    main()
