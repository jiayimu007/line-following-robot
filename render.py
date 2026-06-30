#!/usr/bin/env python3
"""Render a line-follower trajectory CSV into an animated GIF.

The C++ simulator (src/line_follower.cpp) writes one CSV row per frame:

    track,frame,x,y,theta,sensorL_on,sensorR_on

This script reads that log, redraws the same track the simulator followed,
draws the car body, the two sensors, and a small live read-out of each sensor,
then writes an animated GIF with Pillow. It is a renderer only: all of the
control and motion happens in the C++ core. There is no firmware here and none
on the real car.

    python3 render.py build/traj-oval.csv media/demo.gif

The track geometry below is kept in step with src/line_follower.h so the drawn
line matches the line the simulator sensed.
"""

import csv
import math
import sys

import numpy as np
from PIL import Image, ImageDraw

# --- field + car geometry, kept in sync with src/line_follower.h -------------

WIDTH = 760
HEIGHT = 480
TRACK_WIDTH = 22
CAR_LENGTH = 34
CAR_WIDTH = 22
SENSOR_AHEAD = 13
SENSOR_SPREAD = 9

# Colours.
BG = (244, 245, 248)
LINE = (24, 24, 24)
CAR_FILL = (31, 111, 235)
CAR_EDGE = (11, 61, 145)
NOSE = (250, 204, 21)
ON = (34, 197, 94)
OFF = (239, 68, 68)
PANEL = (255, 255, 255)
PANEL_EDGE = (200, 204, 210)
TEXT = (40, 44, 52)


def make_oval(cx, cy, rx, ry, n=600):
    return [(cx + rx * math.cos(i / n * 2 * math.pi),
             cy + ry * math.sin(i / n * 2 * math.pi)) for i in range(n)]


def make_wavy(cx, cy, r, amp, lobes, n=700):
    pts = []
    for i in range(n):
        t = i / n * 2 * math.pi
        rr = r + amp * math.sin(lobes * t)
        pts.append((cx + rr * math.cos(t), cy + rr * math.sin(t)))
    return pts


def make_figure_eight(cx, cy, a, b, n=800):
    pts = []
    for i in range(n):
        t = i / n * 2 * math.pi
        pts.append((cx + a * math.sin(t), cy + b * math.sin(t) * math.cos(t)))
    return pts


def make_track(name):
    if name == "oval":
        return make_oval(380, 240, 300, 170)
    if name == "wavy":
        return make_wavy(380, 240, 175, 55, 5)
    if name == "figure8":
        return make_figure_eight(380, 240, 310, 200)
    raise ValueError("unknown track: " + name)


def read_csv(path):
    """Return (track_name, list_of_frames). Each frame: x, y, theta, sL, sR."""
    frames = []
    track = "oval"
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            track = row["track"]
            frames.append((
                float(row["x"]), float(row["y"]), float(row["theta"]),
                int(row["sensorL_on"]), int(row["sensorR_on"]),
            ))
    return track, frames


def sensor_positions(x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    ax = x + c * SENSOR_AHEAD
    ay = y + s * SENSOR_AHEAD
    px, py = -s, c
    return ((ax - px * SENSOR_SPREAD, ay - py * SENSOR_SPREAD),
            (ax + px * SENSOR_SPREAD, ay + py * SENSOR_SPREAD))


# Supersample factor: everything is drawn at SS times the final resolution and
# then shrunk with a high-quality filter, which antialiases the line and the
# car (Pillow's own drawing is not antialiased).
SS = 3


def draw_track_base(track_pts):
    """Pre-render the static layer (background + line) once, as a base image."""
    img = Image.new("RGB", (WIDTH * SS, HEIGHT * SS), BG)
    d = ImageDraw.Draw(img)
    closed = [(px * SS, py * SS) for px, py in track_pts]
    closed.append(closed[0])
    # The track is a dense polyline, so the straight segments already overlap.
    # `joint="curve"` would add wedges that spike out on convex bends, so instead
    # the line is drawn plain and a filled disc is stamped at every vertex to
    # round the joints smoothly.
    w = TRACK_WIDTH * SS
    d.line(closed, fill=LINE, width=w)
    r = w / 2
    for px, py in closed:
        d.ellipse([px - r, py - r, px + r, py + r], fill=LINE)
    return img


def rotate(cx, cy, dx, dy, c, s):
    return ((cx + dx * c - dy * s) * SS, (cy + dx * s + dy * c) * SS)


def draw_car(d, x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    hl, hw = CAR_LENGTH / 2, CAR_WIDTH / 2
    body = [rotate(x, y, dx, dy, c, s) for dx, dy in
            [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]]
    d.polygon(body, fill=CAR_FILL, outline=CAR_EDGE)
    # nose triangle so the heading is obvious
    nose = [rotate(x, y, dx, dy, c, s) for dx, dy in
            [(hl, 0), (hl - 8, -6), (hl - 8, 6)]]
    d.polygon(nose, fill=NOSE)


def draw_sensor(d, pos, on):
    px, py = pos[0] * SS, pos[1] * SS
    r = 5 * SS
    col = ON if on else OFF
    d.ellipse([px - r, py - r, px + r, py + r], fill=col,
              outline=(255, 255, 255), width=max(1, SS // 2))


def draw_readout(d, sL, sR):
    """Small live panel showing each sensor's on/off state."""
    x0, y0, w, h = 14 * SS, 14 * SS, 168 * SS, 56 * SS
    d.rectangle([x0, y0, x0 + w, y0 + h], fill=PANEL, outline=PANEL_EDGE,
                width=max(1, SS // 2))

    def chip(cx, cy, label, on):
        r = 14 * SS
        col = ON if on else OFF
        d.ellipse([cx, cy, cx + r, cy + r], fill=col, outline=(255, 255, 255))
        txt = "%s: %s" % (label, "ON" if on else "off")
        d.text((cx + 22 * SS, cy + 1 * SS), txt, fill=TEXT, font_size=13 * SS)

    d.text((x0 + 12 * SS, y0 + 6 * SS), "sensors", fill=TEXT, font_size=13 * SS)
    chip(x0 + 16 * SS, y0 + 30 * SS, "L", sL)
    chip(x0 + 92 * SS, y0 + 30 * SS, "R", sR)


def render(csv_path, out_path, stride=12, duration=70):
    """Read the CSV, draw every `stride`-th frame, save an animated GIF."""
    track, frames = read_csv(csv_path)
    base = draw_track_base(make_track(track))

    images = []
    for i in range(0, len(frames), stride):
        x, y, theta, sL, sR = frames[i]
        img = base.copy()
        d = ImageDraw.Draw(img)
        sl_pos, sr_pos = sensor_positions(x, y, theta)
        draw_car(d, x, y, theta)
        draw_sensor(d, sl_pos, sL)
        draw_sensor(d, sr_pos, sR)
        draw_readout(d, sL, sR)
        img = img.resize((WIDTH, HEIGHT), Image.LANCZOS)
        images.append(img.convert("P", palette=Image.ADAPTIVE, colors=64))

    if not images:
        raise SystemExit("no frames to render from " + csv_path)

    images[0].save(out_path, save_all=True, append_images=images[1:],
                   duration=duration, loop=0, optimize=True)

    # Guard against accidentally writing a blank animation.
    arr = np.asarray(images[len(images) // 2].convert("L"))
    if arr.std() < 1.0:
        raise SystemExit("rendered frame looks blank (std=%.3f)" % arr.std())

    print("wrote %s  (%d frames, track=%s)" % (out_path, len(images), track))


def main(argv):
    if len(argv) < 3:
        print("usage: render.py <trajectory.csv> <out.gif>", file=sys.stderr)
        return 1
    render(argv[1], argv[2])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
