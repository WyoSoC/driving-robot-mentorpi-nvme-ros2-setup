"""Patch the factory joystick_control node inside a freshly started MentorPi container.

Bug: update_buttons() polls pygame for gamepad events in `while True:` without ever waiting, so the node
burns a whole CPU core all the time, with or without a gamepad (car 1: 93-99% CPU). Fix: wait 10 ms per
poll (still 100 polls/s, plenty for a gamepad).

Run by run_container.sh: python3 patch_joystick.py [FILE]
"""
import sys

FILE = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/ubuntu/ros2_ws/src/peripherals/peripherals/joystick_control.py"
MARK = "# mentorpi patch: don't busy-wait"
OLD = """            while True:
                for event in pg.event.get():"""
NEW = f"""            while True:
                pg.time.wait(10)  {MARK}
                for event in pg.event.get():"""

src = open(FILE).read()
if MARK in src:
    print("joystick_control already patched")
    sys.exit(0)
if src.count(OLD) != 1:
    sys.exit(f"patch_joystick: factory code changed, cannot apply patch:\n{OLD}")
open(FILE, "w").write(src.replace(OLD, NEW))
print("joystick_control patched: 10 ms wait per gamepad poll")
