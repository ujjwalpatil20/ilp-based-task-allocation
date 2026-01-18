#!/bin/bash
# test_ilp_pipeline.sh

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$SCRIPT_DIR/../../../../.."   # Assuming src/task_management/ilp_learning structure
LOG_DIR="$WORKSPACE_ROOT/ilp_logs"

echo "---------------------------------------------------"
echo "ILP Optimization Pipeline Test"
echo "---------------------------------------------------"
echo "Script Dir: $SCRIPT_DIR"
echo "Log Dir:    $LOG_DIR"
echo "---------------------------------------------------"

if [ ! -d "$LOG_DIR" ]; then
    echo "Error: Log directory not found at $LOG_DIR"
    exit 1
fi

echo "[1/2] Generating Prolog Knowledge Base from latest CSV..."
python3 "$SCRIPT_DIR/generate_ilp_data.py" "$LOG_DIR"

if [ $? -ne 0 ]; then
    echo "Failed to generate data."
    exit 1
fi

echo ""
echo "[2/2] Running Popper to find explanatory rules..."
python3 "$SCRIPT_DIR/run_ilp.py" "$SCRIPT_DIR"

echo "---------------------------------------------------"
echo "Done."
