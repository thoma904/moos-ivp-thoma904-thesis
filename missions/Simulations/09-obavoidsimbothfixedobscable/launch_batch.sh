#!/bin/bash

# Kill entire process tree on Ctrl+C
trap 'echo; echo "Batch cancelled. Killing all processes..."; killall pAntler 2>/dev/null; kill -- -$$ 2>/dev/null; exit 1' SIGINT

# Fixed obstacle configuration for this test
OBSTACLE_AMT=8
OBSTACLE_SEP=10

NUM_RUNS=100
MAX_RETRIES=3

# Fixed start position: 30m to the right of first waypoint (40,-30)
FIXED_START="x=70,y=-16,heading=270"

# pwt_outer and pwt_inner distances to test (lockstep, increase by 5)
OUTER_VALS=(15 20 25 30)
INNER_VALS=(5  10 15 20)

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

# Write fixed start position so init_field.sh won't randomize
echo "$FIXED_START" > vpositions.txt

# Outer loop: vary pwt distances
for ((d=0; d<${#OUTER_VALS[@]}; d++)); do
    PWT_OUTER=${OUTER_VALS[$d]}
    PWT_INNER=${INNER_VALS[$d]}

    echo "============================================================"
    echo "  Testing pwt_outer=$PWT_OUTER, pwt_inner=$PWT_INNER"
    echo "============================================================"

    for ((j=1; j<=NUM_RUNS; j++)); do
        echo "=== Run $j/$NUM_RUNS | outer=$PWT_OUTER, inner=$PWT_INNER ==="

        SUCCESS=false
        for ((retry=1; retry<=MAX_RETRIES; retry++)); do
            if [ $retry -gt 1 ]; then
                echo "  Retry $retry/$MAX_RETRIES"
                kill_all_moos
                ./clean.sh
            fi

            ./launch_sleep.sh --obs="$OBSTACLE_AMT" --sep="$OBSTACLE_SEP" \
                --pwt_outer="$PWT_OUTER" --pwt_inner="$PWT_INNER" --nogui 10
            EXIT_CODE=$?

            if [ $EXIT_CODE -eq 0 ] && ls LOG_*/ 1>/dev/null 2>&1; then
                SUCCESS=true
                break
            else
                echo "  Run failed (exit code $EXIT_CODE). Cleaning up..."
            fi
        done

        if [ "$SUCCESS" = true ]; then
            tar -czf "${RESULTS_DIR}/obavoidsim_outer${PWT_OUTER}_inner${PWT_INNER}_run_$(printf '%03d' $j).tar.gz" LOG_*/ XLOG_*/
        else
            echo "  WARNING: Run $j failed after $MAX_RETRIES retries. Skipping."
        fi

        # Thorough cleanup between runs
        kill_all_moos
        ./clean.sh

        # Re-write fixed start position (clean.sh may remove it)
        echo "$FIXED_START" > vpositions.txt
    done
done

echo "Batch complete: ${#OUTER_VALS[@]} distance settings x $NUM_RUNS runs each"
echo "Results saved in $RESULTS_DIR/"