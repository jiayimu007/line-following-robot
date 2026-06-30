# Bill of materials

The line follower is a single PCB. The first group lists the parts a builder fits and
solders; the second lists the signal-path parts that come pre-populated on the board.
There is **no microcontroller** — the steering is decided in analog hardware.

## Builder-fitted parts

| # | Part | Qty | Purpose |
|---|------|-----|---------|
| 1 | PCB mainboard | 1 | chassis + circuit |
| 2 | Photoresistor / LDR (≈GL5516) | 2 | front-facing line sensors |
| 3 | Indicator LED | 2 | shows each comparator's state |
| 4 | Diode | 2 | motor flyback protection |
| 5 | DC gear motor | 2 | drives one wheel each |
| 6 | Wheel (hub + tyre) | 2 | differential drive |
| 7 | AA battery holder | 1 | ~3 V supply |
| 8 | AA battery | 2 | power |
| 9 | Power switch | 1 | on/off |
| 10 | Screws / nuts / standoff | set | mounting + front support |
| 11 | Track sheet (black line on white) | 1 | test track |

## Pre-populated on the board

| Part | Qty | Purpose |
|------|-----|---------|
| LM393 dual comparator | 1 | thresholds both sensors (one comparator each) |
| Trimmer potentiometer | 2 | sets each sensor's switching threshold |
| NPN transistor | 2 | motor driver (one per motor) |
| Pull-up resistor (≈4.7k–10k) | 2 | pulls each LM393 open-collector output high |
| Sensor divider resistor | 2 | forms the voltage divider with each LDR |
| Base resistor | 2 | limits each transistor's base current |
| LED series resistor | 2 | limits each indicator-LED current |
| Decoupling capacitor (0.1 µF) | 1 | steadies the LM393 supply |

**Supply:** 2 × AA in series, ≈ 3 V.

> The LM393 has an **open-collector** output (it can only pull low), so each output
> needs a pull-up resistor to +3 V — see [hardware/schematic.svg](schematic.svg).
