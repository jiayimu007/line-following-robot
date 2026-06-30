# Line-Following Robot

*English | [中文](README.zh-CN.md)*

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A small two-wheeled robot that follows a black line on the floor. It steers with two light
sensors and an analog comparator, and there is no microcontroller on board, so nothing runs
in software on the car itself. I built and calibrated the kit, and I wrote the simulation in
this repo so you can watch the control loop without the hardware.

![Line follower on a curved track](media/demo.gif)

## How it works

Two light sensors (LDRs) point down at the floor from the front of the car. Each one feeds
one half of an LM393 comparator, which compares it against a threshold you set with a
trimmer and gives a clean on/off signal. The two channels are cross-wired, so each sensor
controls the motor on the opposite side. When the car drifts and a sensor slips off the
line, the far motor eases off and the car turns back. All of this happens in the analog
circuit; there is no code involved.

For a longer explanation see [docs/how-it-works.md](docs/how-it-works.md), and the full
circuit is in [hardware/schematic.svg](hardware/schematic.svg).

## The simulation

Since the car decides everything in hardware, I rebuilt the same idea in software to make
the loop easy to see. It is a software model of the control principle, not firmware for a
real car; the real car has no firmware at all.

The model is written in C++ (`src/`). Two sensors read a drawn line, a simple on/off rule
picks the two wheel speeds, and a basic differential-drive model moves the car. The
simulator runs each track and writes a trajectory log (one row per frame: position, heading
and the two sensor states). A Python script (`render.py`, using Pillow and numpy) reads that
log, redraws the track and the car, and saves the animated GIF above. There are three tracks
to choose from: an oval, a wavy one, and a figure-8 that crosses itself.

One behaviour only exists in the simulation: if the car loses the line on a tight bend, it
keeps turning the way it last turned until it picks the line back up. The real board has no
memory, so it cannot do that.

## The browser demo

There is also a small in-browser player under [`web/`](web/). It is **animation only**: the
C++ engine runs each track and exports its trajectory to a JSON file
(`web/data/oval.json`, `wavy.json`, `figure8.json`), and the player just loads that data and
draws it on a canvas frame by frame. The page does **not** re-run the simulation in
JavaScript; there is no sensor model, comparator, controller or kinematics in the player. It
reads the precomputed car pose and the two precomputed sensor states and renders them, with
a track selector, play/pause, a speed slider and a live L/R sensor read-out.

The data files come straight from the engine:

```bash
make web      # writes web/data/{oval,wavy,figure8}.json with the C++ simulator
```

Then open `web/index.html` (or the repo root `index.html`, which redirects to it). The same
JSON the player animates is produced by the same simulation step that feeds the GIF, so the
two always agree.

### Building and running

You need a C++17 compiler and Python with Pillow and numpy. With `make`:

```bash
make test     # build and run the headless test
make web      # export web/data/*.json for the browser player
make gif      # rebuild media/demo.gif and media/demo-figure8.gif
```

Or by hand:

```bash
c++ -O2 -std=c++17 tests/test.cpp -o build/test && ./build/test
c++ -O2 -std=c++17 src/line_follower.cpp -o build/line_follower
./build/line_follower oval 1400 build/traj-oval.csv     # CSV for the GIF renderer
./build/line_follower oval 1400 web/data/oval.json      # JSON for the browser player
python3 render.py build/traj-oval.csv media/demo.gif
```

A path ending in `.json` makes the simulator write the player's JSON (track polyline plus the
per-frame trajectory); any other path writes the CSV the Python renderer reads. Both come from
the identical simulation step.

The test runs the same control and motion code headlessly. It checks that the car stays
within a few pixels of the line on all three tracks, and it pins down the turn direction in
world coordinates so a mirror-flipped version of the model would fail.

## Hardware

<img src="hardware/schematic.svg" alt="Schematic" width="640">

The schematic draws one channel. The other one is identical, with the right sensor driving
the left motor, and both share the single LM393 and the 3 V supply. The parts are listed in
[hardware/bom.md](hardware/bom.md).

| | |
|---|---|
| ![Board, top](media/car-board-top.jpg) | ![Front sensors](media/car-front-sensors.jpg) |
| ![Side](media/car-side.jpg) | ![On the track](media/car-on-track.jpg) |

## Build

The assembly and calibration steps are in [docs/build-guide.md](docs/build-guide.md).

## Layout

```
src/        the C++ simulation core (line_follower.h, line_follower.cpp)
tests/      the headless C++ test
render.py   the Python (Pillow) renderer that draws the GIF
web/        the browser player (index.html, player.js, style.css) + data/*.json
index.html  redirect to web/ for the static site
Makefile    build + test + data + web + render targets
hardware/   schematic and parts list
docs/       how it works, build guide
media/      photos and the demo clips
```

## License

MIT, see [LICENSE](LICENSE). Made by Jiayi Mu ([github.com/jiayimu007](https://github.com/jiayimu007)).
