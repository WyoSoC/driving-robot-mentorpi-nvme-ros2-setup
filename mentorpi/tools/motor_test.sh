#!/bin/bash
# Drive each rear motor on its own in speed steps. WHEELS OFF THE GROUND (car on a box).
#   tools/motor_test.sh [left|right|both] [STEP_SECONDS]   (default: both, 3 s per step)
set -euo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)
docker exec -i -u ubuntu MentorPi /bin/zsh -c "source ~/.zshrc >/dev/null 2>&1; exec python3 -u - ${1:-both} ${2:-3}" \
    < "$DIR/motor_test.py"
