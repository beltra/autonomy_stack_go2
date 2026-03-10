#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "[Deprecated] Use ./sim.sh"
exec "$SCRIPT_DIR/sim.sh" "$@"
