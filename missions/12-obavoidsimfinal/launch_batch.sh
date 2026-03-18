#!/bin/bash

# Kill entire process tree on Ctrl+C
trap 'echo; echo "Batch cancelled. Killing all processes..."; killall pAntler 2>/dev/null; kill -- -$$ 2>/dev/null; exit 1' SIGINT

# Fixed obstacle configuration for this test
OBSTACLE_AMT=8
OBSTACLE_SEP=10

NUM_RUNS=100
MAX_RETRIES=3

# Top 4 configurations: "tow_outer,tow_inner,veh_outer,veh_inner"
CONFIGS=(
    "20,10,30,20"
    "25,15,30,20"
    "20,10,25,15"
    "50,40,35,25"
    "40,15,40,5"
)

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

# Loop over the 4 specific configurations
for CONFIG in "${CONFIGS[@]}"; do
    IFS=',' read -r TOW_PWT_OUTER TOW_PWT_INNER VEH_PWT_OUTER VEH_PWT_INNER <<< "$CONFIG"

    echo "============================================================"
    echo "  Tow: outer=$TOW_PWT_OUTER inner=$TOW_PWT_INNER"
    echo "  Veh: outer=$VEH_PWT_OUTER inner=$VEH_PWT_INNER"
    echo "============================================================"

    for ((j=1; j<=NUM_RUNS; j++)); do
        TARFILE="${RESULTS_DIR}/obavoidsim_touter${TOW_PWT_OUTER}_tinner${TOW_PWT_INNER}_vouter${VEH_PWT_OUTER}_vinner${VEH_PWT_INNER}_run_$(printf '%03d' $j).tar.gz"
        if [ -f "$TARFILE" ]; then
            echo "=== Run $j/$NUM_RUNS | tow=$TOW_PWT_OUTER/$TOW_PWT_INNER veh=$VEH_PWT_OUTER/$VEH_PWT_INNER === SKIPPED (exists)"
            continue
        fi

        echo "=== Run $j/$NUM_RUNS | tow=$TOW_PWT_OUTER/$TOW_PWT_INNER veh=$VEH_PWT_OUTER/$VEH_PWT_INNER ==="

        SUCCESS=false
        for ((retry=1; retry<=MAX_RETRIES; retry++)); do
            if [ $retry -gt 1 ]; then
                echo "  Retry $retry/$MAX_RETRIES"
                kill_all_moos
                ./clean.sh
            fi

            ./launch_sleep.sh --obs="$OBSTACLE_AMT" --sep="$OBSTACLE_SEP" \
                --tow_pwt_outer="$TOW_PWT_OUTER" --tow_pwt_inner="$TOW_PWT_INNER" \
                --veh_pwt_outer="$VEH_PWT_OUTER" --veh_pwt_inner="$VEH_PWT_INNER" \
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
done

echo "Batch complete: 4 configurations x $NUM_RUNS runs each (random start positions)"
echo "Results saved in $RESULTS_DIR/"
