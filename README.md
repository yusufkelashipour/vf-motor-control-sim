# V/f (Volts-per-Hertz) Motor Control Simulation (C)

A C program simulating the core control strategy used by **variable frequency drives (VFDs)** to control AC induction motor speed: as the commanded frequency ramps up, output voltage is scaled proportionally to keep magnetic flux — and therefore available torque — roughly constant across the operating range.

## Why this project

VFD firmware doesn't just "turn the motor on" — it has to ramp speed up smoothly, scale voltage correctly at every frequency, and compensate for motor behavior at low speed. This program computes that control logic — the **V/f control law**, the acceleration **ramp profile**, and the **low-frequency voltage boost** — directly in C, the same language real motor-drive firmware is written in, and prints the resulting values the way firmware would log them (e.g. over serial/debug output) rather than relying on any external plotting tool.

## What it does

- Ramps a commanded frequency linearly from 0 Hz to 60 Hz over 5 seconds (mimicking a VFD acceleration ramp, a tunable firmware parameter used to avoid excessive inrush current and mechanical shock)
- Computes commanded voltage at each instant using the V/f law: `V = (V_rated / f_rated) * f`
- Adds a **voltage boost** below 10 Hz that tapers to zero, compensating for the voltage drop across motor winding resistance at low speed — without this, a motor can stall near standstill even under V/f control
- Models motor speed response with a first-order lag (approximating mechanical inertia and slip) rather than assuming the motor instantly matches the commanded frequency
- Prints a formatted table sampling the simulation roughly once per second: commanded frequency, commanded voltage, resulting V/f ratio, and actual motor speed

## Sample output

```
Time(s) | Freq_cmd(Hz) | Voltage_cmd(V) | V/f ratio | Motor speed(Hz)
------------------------------------------------------------------------
   0.00 |         0.00 |          25.00 |        -- |            0.00
   1.01 |        12.12 |          92.92 |      7.67 |            6.34
   2.01 |        24.12 |         184.92 |      7.67 |           17.28
   3.01 |        36.12 |         276.92 |      7.67 |           29.08
   4.01 |        48.12 |         368.92 |      7.67 |           41.05
   5.00 |        60.00 |         460.00 |      7.67 |           52.92
   6.00 |        60.00 |         460.00 |      7.67 |           58.68
   7.00 |        60.00 |         460.00 |      7.67 |           59.75
```

The V/f ratio locks to a constant ~7.67 V/Hz almost immediately after the low-frequency boost region — confirming the control law is working as intended. Motor speed lags slightly behind the commanded frequency throughout, reflecting simulated mechanical inertia.

## Files

| File | Description |
|---|---|
| `vf_control_sim.c` | The complete C program |

## How to run

**Requires:** `gcc` (or any C compiler)

```bash
gcc -Wall -Wextra -o vf_control_sim vf_control_sim.c -lm
./vf_control_sim
```

Compiles cleanly with zero warnings under `-Wall -Wextra`.


