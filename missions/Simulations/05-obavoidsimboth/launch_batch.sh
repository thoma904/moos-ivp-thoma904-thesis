#!/bin/bash

# Kill entire process tree on Ctrl+C
trap 'echo; echo "Batch cancelled. Killing all processes..."; killall pAntler 2>/dev/null; kill -- -$$ 2>/dev/null; exit 1' SIGINT

# Amount of obstacles to test with corresponding separation distances
# For <=10 obstacles, sep=10 works fine. For >10, lower sep to fit in same area.
options=("1" "2" "4" "8" "10" "12" "16" "20")
seps=("10" "10" "10" "10" "10" "8"  "6"  "5")

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

# Launch command with each option
for ((j=1; j<=NUM_RUNS; j++)); do
    for i in "${!options[@]}"; do
        opt="${options[$i]}"
        sep="${seps[$i]}"
        echo "=== Run $j/$NUM_RUNS | obs=$opt, sep=$sep ==="

        SUCCESS=false
        for ((retry=1; retry<=MAX_RETRIES; retry++)); do
            if [ $retry -gt 1 ]; then
                echo "  Retry $retry/$MAX_RETRIES for obs=$opt, sep=$sep"
                kill_all_moos
                ./clean.sh
            fi

            ./launch_sleep.sh --obs="$opt" --sep="$sep" --nogui 10
            EXIT_CODE=$?

            if [ $EXIT_CODE -eq 0 ] && ls LOG_*/ 1>/dev/null 2>&1; then
                SUCCESS=true
                break
            else
                echo "  Run failed (exit code $EXIT_CODE). Cleaning up..."
            fi
        done

        if [ "$SUCCESS" = true ]; then
            tar -czf "${RESULTS_DIR}/obavoidsim_obs_${opt}_sep_${sep}_run_$(printf '%03d' $j).tar.gz" LOG_*/ XLOG_*/
        else
            echo "  WARNING: Run $j obs=$opt sep=$sep failed after $MAX_RETRIES retries. Skipping."
        fi

        # Thorough cleanup between runs
        kill_all_moos
        ./clean.sh
    done
done

echo "Batch complete: $NUM_RUNS runs x ${#options[@]} configs = $((NUM_RUNS * ${#options[@]})) total simulations"
echo "Results saved in $RESULTS_DIR/"