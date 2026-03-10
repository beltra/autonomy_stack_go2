#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "[Deprecated] Use ./real_rviz.sh"
exec "$SCRIPT_DIR/real_rviz.sh" "$@"
