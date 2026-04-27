#!/bin/bash
# Run analyze_friction.py on all experiment dates newer than 26_04_14-14_49_13.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="$SCRIPT_DIR/../data"
CUTOFF="26_04_14-14_00_00"

for dir in "$DATA_DIR"/*/; do
    date_name="$(basename "$dir")"
    if [[ "$date_name" > "$CUTOFF" ]]; then
        echo "=== Analyzing $date_name ==="
        python3 "$SCRIPT_DIR/analyze_friction.py" --date "$date_name"
        echo ""
    fi
done

echo "Done."
