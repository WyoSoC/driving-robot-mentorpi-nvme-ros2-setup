"""Patch the factory ros_robot_controller node inside a freshly started MentorPi container.

Run by run_container.sh on every container start (the container is recreated from the image
each time, so the patch is never applied twice to the same file, but it is idempotent anyway).

1. PWM port map: the software always drives the steering servo as PWM servo 3 (odom_publisher,
   lidar_controller, joystick apps, ...). Cars whose steering servo is plugged into another
   port of the RRC Lite board get a map from logical id to physical port, applied where the
   node forwards PWM commands to the board. With STEER_SERVO_PORT=4 the map is {3: 4, 4: 3}
   (a swap, so nothing else ends up on the steering port).
2. get_pwm_servo_state service: the factory handler has the wrong signature and crashes the
   whole controller node when called (and its board read can block forever). Replaced by a
   handler that answers success=False immediately.

Usage: python3 patch_controller.py STEER_SERVO_PORT [NODE_FILE]
"""
import sys

NODE = "/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py"
MARK = "# --- mentorpi patch ---"

port = int(sys.argv[1])
if len(sys.argv) > 2:
    NODE = sys.argv[2]
if not 1 <= port <= 4:
    sys.exit(f"STEER_SERVO_PORT must be 1-4, got {port}")
port_map = {} if port == 3 else {3: port, port: 3}

src = open(NODE).read()
if MARK in src:  # already patched: drop the old map line, re-apply below
    src = src.split(MARK + "\n", 1)[0] + src.split(MARK + "\n", 1)[1].split("\n", 1)[1]

replacements = [
    # 1. map logical PWM ids to physical ports
    ("""            if i.id and i.position:
                data.extend([[i.id[0], i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])""",
     """            if i.id and i.position:
                data.extend([[PWM_PORT_MAP.get(i.id[0], i.id[0]), i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(PWM_PORT_MAP.get(i.id[0], i.id[0]), i.offset[0])"""),
    # 3. servo offsets (calibration/servo_config.yaml, key = logical id; 3 = steering) to the right port
    ("""                    self.board.pwm_servo_set_offset(servo_id, offset)""",
     """                    self.board.pwm_servo_set_offset(PWM_PORT_MAP.get(servo_id, servo_id), offset)"""),
    # 2. harmless get_pwm_servo_state
    ("""    def get_pwm_servo_state(self, msg):
        states = []""",
     """    def get_pwm_servo_state(self, request, response):
        # Factory handler crashed the node (wrong signature) and could block on the board read.
        response.success = False
        return response

    def _factory_get_pwm_servo_state(self, msg):
        states = []"""),
]
for old, new in replacements:
    if new in src:
        continue
    if src.count(old) != 1:
        sys.exit(f"patch_controller: factory code changed, cannot apply patch:\n{old}")
    src = src.replace(old, new)

# PWM_PORT_MAP definition just before the first top-level class (imports may span lines)
lines = src.split("\n")
first_class = next(i for i, l in enumerate(lines) if l.startswith("class "))
lines.insert(first_class, f"{MARK}\nPWM_PORT_MAP = {port_map!r}  # logical PWM servo id -> RRC Lite port\n")
open(NODE, "w").write("\n".join(lines))
print(f"ros_robot_controller patched: steering servo (id 3) -> port {port}, PWM_PORT_MAP={port_map}")
