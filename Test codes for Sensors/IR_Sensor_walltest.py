#!/usr/bin/env python3
"""
Test all 4 FC-51 IR wall sensors on the Raspberry Pi 5 - gpiod v2 API.
FC-51 output: LOW (0) = obstacle detected, HIGH (1) = clear.

Wiring: VCC -> 3.3V, GND -> GND, OUT -> GPIO below.
Edit SENSORS if your pins differ. Press Stop in Thonny to end.
"""
import glob
import time
import gpiod
from gpiod.line import Direction, Value, Bias

SENSORS = {
    "FRONT_L": "GPIO17",
    "FRONT_R": "GPIO27",
    "LEFT":    "GPIO22",
    "RIGHT":   "GPIO23",
}
POLL_S = 0.05  # 20 Hz


def find_chip_with(name):
    """Pi 5 header pins are on gpiochip0 (new kernels) or gpiochip4 (old)."""
    for path in sorted(glob.glob("/dev/gpiochip*")):
        try:
            with gpiod.Chip(path) as chip:
                return path, chip.line_offset_from_id(name)
        except (OSError, ValueError):
            continue
    raise SystemExit(f"Could not find {name} on any gpiochip")


chip_path, _ = find_chip_with("GPIO17")
offsets = {}
with gpiod.Chip(chip_path) as chip:
    for name, pin in SENSORS.items():
        offsets[name] = chip.line_offset_from_id(pin)
        print(f"{name:8s} -> {pin} (offset {offsets[name]}) OK")
print(f"Using {chip_path}")

settings = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
request = gpiod.request_lines(
    chip_path,
    consumer="ir_test",
    config={tuple(offsets.values()): settings},
)

print("\nWave your hand / a box in front of each sensor.\n")
hits = {n: 0 for n in SENSORS}
last = None
try:
    while True:
        state = {n: request.get_value(o) == Value.INACTIVE for n, o in offsets.items()}
        if state != last:
            for n, blocked in state.items():
                if blocked and (last is None or not last[n]):
                    hits[n] += 1
            row = "  ".join(f"{n}:{'WALL' if b else '----'}" for n, b in state.items())
            print(f"{time.strftime('%H:%M:%S')}  {row}   hits={hits}")
            last = state
        time.sleep(POLL_S)
except KeyboardInterrupt:
    pass
finally:
    request.release()
    print("Released GPIO lines.")