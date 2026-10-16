#!/bin/bash
# Steering check: lift the front wheels, then run on the car. Default sequence: full left 2 s,
# full right 2 s, centre. Custom: steer_test.sh 20:2 -20:2 0:1   (degrees:seconds, + = left)
set -euo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)
[ $# -gt 0 ] || set -- 40:2 -40:2 0:1
docker exec -i -u ubuntu MentorPi /bin/zsh -c "source ~/.zshrc >/dev/null 2>&1; exec python3 - $*" < "$DIR/steer_test.py"
