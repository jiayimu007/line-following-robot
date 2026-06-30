# Build guide

*English | [中文](build-guide.zh-CN.md)*

This is the assembly and calibration order for the car. Parts are in
[../hardware/bom.md](../hardware/bom.md); the circuit is in
[../hardware/schematic.svg](../hardware/schematic.svg).

## 1. Board and harness

The signal-path parts are soldered to the mainboard: the LM393 socket, the transistors,
the resistors, the LEDs, and the sensor, motor, battery and switch leads. Work on a clean,
dry surface and try not to tug the thin sensor and motor wires.

## 2. Power

Fix the **battery holder** to the underside of the board and fit **2 × AA** cells
(~3 V). Keep the **power switch** off until the build is finished.

## 3. Motors and wheels

Mount the two **DC gear motors** on the board so their output shafts are on one
straight line, then press a **wheel** onto each shaft. The front of the chassis rests
on a small standoff/skid, giving a stable three-point stance.

## 4. The LM393

Seat the **LM393** in its socket the right way round, with pin 1 (the notch or dot)
matching the socket, and make sure **every pin goes into its hole**. A bent or half-seated
pin shorts the chip and can destroy it.

## 5. Sensors

The two **photoresistors** face down at the front, close together so that both sit over
the line when the car is centered; as the car drifts, the line slips out from under one
of them and the car steers back toward the line. Their height above the surface affects
sensitivity (see calibration).

## 6. First power-up and calibration

1. Place the car on the track so both sensors sit on the line.
2. Turn the switch on. The **indicator LEDs** show what each comparator sees.
3. Turn each **trimmer potentiometer** until that side reliably distinguishes the
   black line from the white surface (the LED switches cleanly as the sensor crosses
   the edge).
4. Let the car run. If it leaves the line on curves, fine-tune the trimmers, the
   sensor height, or the line width until it tracks smoothly.

## Handling

The sensor and motor wires are soldered to the board and are easy to stress, so handle the
car gently and keep it dry. Take the batteries out for long-term storage so they don't
corrode the holder.
