#!/bin/bash

# Kill entire process tree on Ctrl+C
trap 'echo; echo "Batch cancelled. Killing all processes..."; killall pAntler 2>/dev/null; kill -- -$$ 2>/dev/null; exit 1' SIGINT

# Fixed obstacle configuration for this test
OBSTACLE_AMT=8
OBSTACLE_SEP=10

NUM_RUNS=100
MAX_RETRIES=3

# Create output directory for results
RESULTS_DIR="batch_results"
mkdir -p "$RESULTS_DIR"

#------------------------------------------------------------
#  Function: kill_all_moos
#  Ensure all MOOS processes are dead and ports are released
#------------------------------------------------------------
kill_all_moos() {
    ktm
    sleep 2
}

for ((j=1; j<=NUM_RUNS; j++)); do
    TARFILE="${RESULTS_DIR}/obavoidsim_run_$(printf '%03d' $j).tar.gz"
    if [ -f "$TARFILE" ]; then
        echo "=== Run $j/$NUM_RUNS === SKIPPED (exists)"
        continue
    fi

    echo "=== Run $j/$NUM_RUNS ==="

    SUCCESS=false
    for ((retry=1; retry<=MAX_RETRIES; retry++)); do
        if [ $retry -gt 1 ]; then
            echo "  Retry $retry/$MAX_RETRIES"
            kill_all_moos
            ./clean.sh
        fi

        ./launch_sleep.sh --obs="$OBSTACLE_AMT" --sep="$OBSTACLE_SEP" \
            --rand --nogui 10
        EXIT_CODE=$?

        if [ $EXIT_CODE -eq 0 ] && ls LOG_*/ 1>/dev/null 2>&1; then
            SUCCESS=true
            break
        else
            echo "  Run failed (exit code $EXIT_CODE). Cleaning up..."
        fi
    done

    if [ "$SUCCESS" = true ]; then
        tar -czf "$TARFILE" LOG_*/ XLOG_*/
    else
        echo "  WARNING: Run $j failed after $MAX_RETRIES retries. Skipping."
    fi

    # Thorough cleanup between runs
    kill_all_moos
    ./clean.sh
done

echo "Batch complete: $NUM_RUNS runs (random start positions)"
echo "Results saved in $RESULTS_DIR/"
