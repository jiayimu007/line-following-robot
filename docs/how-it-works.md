# How it works

*English | [中文](how-it-works.zh-CN.md)*

The robot follows a black line on a light surface. There is no microcontroller. The whole
control loop is just a light sensor, a comparator and a motor on each side.

## 1. Sensing the line

Each side of the car carries a **photoresistor (LDR)** pointing down at the surface. A
black line reflects little light and a white surface reflects a lot, so the LDR's
resistance changes depending on whether it is over the line.

The LDR forms a **voltage divider** with a fixed resistor, which turns "how much light"
into a voltage:

```
+3V ──[ LDR ]──┬── sensor voltage  → comparator
              [ R ]
               │
              GND
```

When the sensor moves over the dark line, less light reaches the LDR, its resistance
rises, and the sensor voltage shifts. That shift is what the comparator detects.

## 2. Deciding: the comparator

The sensor voltage goes to the **inverting (−) input** of an **LM393 comparator**. The
**non-inverting (+) input** holds a reference voltage set by a **trimmer potentiometer**.
The comparator just asks whether the sensor is brighter or darker than that threshold and
gives a clean on/off signal. A sensor over the line is darker, so its voltage is low and
the output goes high. The trimmer is there so you can set the threshold for the lighting
and the particular track.

The LM393 is a **dual** comparator, so one chip handles both sides. It does the same job
as reading a sensor and running an `if` statement, except it happens in hardware,
instantly, with no code.

## 3. Acting: the motors

The LM393 output is **open-collector**, meaning it can only pull low, so a **pull-up
resistor** to +3 V sets its high level. Each comparator output then drives a motor through
a small **NPN transistor**, since the comparator can't power a motor on its own. The two
channels are **cross-wired**: each sensor's comparator drives the motor on the *opposite*
side, which is what makes the loop correct itself (see the next section). A **flyback
diode** across the motor protects the transistor from the voltage spike when the motor
switches off, and an **indicator LED** shows each comparator's output. The LED lights when
that sensor is off the line.

## 4. The closed loop

The two sensors sit close together at the front. When the car is centered on the line,
both of them are over it and both motors run, so the car goes straight. When it drifts:

| Situation | Sensors | Result |
|---|---|---|
| Centered | both on the line | both motors run → straight |
| Drifted right, line under left sensor only | left on, right off | the right sensor (off) cuts the cross-wired **left** motor → car pivots **left**, back to the line |
| Drifted left, line under right sensor only | right on, left off | the left sensor (off) cuts the cross-wired **right** motor → car pivots **right** |

Slowing the wheel on the side the car needs to turn toward pulls it back onto the line,
so the error keeps correcting itself. This is **bang-bang control**: each motor is either
fully on or fully off, never in between. It's the simplest kind of closed loop, and it's
all the comparators are doing.

## 5. From hardware to the simulation

Because the real decision happens in analog hardware, the simulation in this repo rebuilds
the same loop in software so you can watch it. It is a software model of the control
principle, not on-board firmware, because there is no firmware on the car.

The model is written in C++ (`src/line_follower.h` and `src/line_follower.cpp`). Two virtual
sensors read a drawn line, the on/off rule sets the two wheel speeds, and a differential-drive
model moves the car. The simulator records the trajectory to a CSV log, and a Python script
(`render.py`, with Pillow) draws the track and the car frame by frame into the demo GIF.

In the simulation the rule is written as *slow the wheel on the side still over the line*.
For two sensors that comes out the same as the board's cross-wiring (cutting the motor
opposite the sensor that left the line); it is just described from the wheel's side instead
of the wiring's.

One thing exists only in the simulation: if the car briefly loses the line on a sharp
bend, it keeps turning the way it last turned until it finds the line again. That is just a
software convenience. The analog board has no memory, so it can't do it.

A headless test (`tests/test.cpp`) runs the same control and motion code with no rendering.
It checks that the car stays within a few pixels of the line on all three tracks, and it
pins down the turn direction in world coordinates so a mirror-flipped model would fail.

## 6. The browser player

The repo also has a small in-browser player (`web/`). It does **not** re-run the simulation.
The C++ engine exports each track's trajectory to JSON (`make web` writes
`web/data/oval.json`, `wavy.json` and `figure8.json`), and the player loads that JSON and
animates it on a `<canvas>`. The simulator writes the JSON itself: a path ending in `.json`
makes it emit the track polyline plus a per-frame trajectory `[{x, y, theta, sLx, sLy, sRx,
sRy, onL, onR}]`, while any other path keeps the CSV the Python renderer reads. Both formats
come from the identical simulation step.

So everything substantive happens in the C++ engine. The player contains no sensor model, no
comparator, no controller and no kinematics: it draws the precomputed track line, blends two
stored poses for smooth motion, and stamps the two precomputed sensor dots with their stored
on/off states. It is the same role `render.py` plays for the GIF, moved into the browser.

The circuit (one of the two identical channels) is in
[../hardware/schematic.svg](../hardware/schematic.svg).
