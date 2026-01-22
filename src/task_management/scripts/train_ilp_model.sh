#!/bin/bash
set -e

# Define paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MGMT_DIR="$(dirname "$SCRIPT_DIR")"
ILP_LEARNING_DIR="$TASK_MGMT_DIR/ilp_learning"

echo "==========================================="
echo "       ILP MODEL TRAINING PIPELINE         "
echo "==========================================="

# 1. Generate Data from Logs
echo "[1/2] Generating Prolog facts from latest log..."
cd "$ILP_LEARNING_DIR"
python3 generate_ilp_data.py

# 2. Run Popper to Learn Rules
echo ""
echo "[2/2] Learning rules with Popper..."
python3 run_ilp.py

echo ""
echo "==========================================="
echo "              TRAINING COMPLETE            "
echo "==========================================="
echo "Check '$TASK_MGMT_DIR/learned_rules.pl' for the result."
