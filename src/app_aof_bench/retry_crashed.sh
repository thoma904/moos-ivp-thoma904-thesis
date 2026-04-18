#!/bin/bash
#---------------------------------------------------------
# retry_crashed.sh — Re-run only CRASHED entries from a sweep CSV
#
# Usage: ./retry_crashed.sh <sweep_csv>
#   e.g.: ./retry_crashed.sh results/sweep_20260404_122813.csv
#
# Parses CRASHED rows, reconstructs the aof_bench command,
# and patches the CSV in place with successful results.
#---------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BENCH="$REPO_ROOT/bin/aof_bench"

if [ ! -x "$BENCH" ]; then
  echo "Error: aof_bench not found at $BENCH. Build first."
  exit 1
fi

if [ -z "$1" ]; then
  echo "Usage: $0 <sweep_csv>"
  echo "  Retries all CRASHED entries in the given CSV file."
  exit 0
fi

CSVFILE="$1"
if [ ! -f "$CSVFILE" ]; then
  echo "Error: $CSVFILE not found"
  exit 1
fi

REPS=50
MAX_RETRIES=3

# Map sweep parameter names back to aof_bench flags
build_args() {
  local sweep="$1"
  local value="$2"

  case "$sweep" in
    sim_horizon)    echo "--sim_horizon=$value --sim_dt=0.1" ;;
    sim_dt)         echo "--sim_horizon=25 --sim_dt=$value" ;;
    cable_length)   echo "--cable_len=$value" ;;
    cable_check)    echo "--cable_check=$value" ;;
    cable_start)    echo "--cable_start=$value" ;;
    turn_rate)      echo "--turn_rate=$value" ;;
    cable_dyn)
      if [ "$value" = "true" ]; then echo "--cable_dyn=true"
      else echo "--cable_dyn=false"; fi ;;
    side_lock)
      if [ "$value" != "off" ]; then echo "--side_lock=$value"; fi ;;
    domain_res)
      # value is like "72x11"
      local crs=$(echo "$value" | cut -dx -f1)
      local spd=$(echo "$value" | cut -dx -f2)
      echo "--crs_pts=$crs --spd_pts=$spd" ;;
    config)
      case "$value" in
        baseline)
          echo "--turn_rate=15 --sim_horizon=25 --sim_dt=0.2 --crs_pts=72 --spd_pts=21 --cable_dyn=true" ;;
        best_collision)
          echo "--turn_rate=0 --sim_horizon=25 --sim_dt=0.2 --crs_pts=72 --spd_pts=21 --cable_dyn=false --side_lock=port" ;;
        lowest_timeout)
          echo "--turn_rate=0 --sim_horizon=25 --sim_dt=0.2 --crs_pts=72 --spd_pts=21 --cable_dyn=false" ;;
        reflector_360)
          echo "--turn_rate=0 --sim_horizon=25 --sim_dt=0.2 --crs_pts=360 --spd_pts=21 --cable_dyn=true" ;;
        *) echo "" ;;
      esac ;;
    *) echo "" ;;
  esac
}

# Find and retry crashed entries
CRASHED_LINES=$(grep "CRASHED" "$CSVFILE")
N_CRASHED=$(echo "$CRASHED_LINES" | grep -c "CRASHED")

if [ "$N_CRASHED" -eq 0 ]; then
  echo "No CRASHED entries found in $CSVFILE"
  exit 0
fi

echo "Found $N_CRASHED crashed entries. Retrying (up to $MAX_RETRIES attempts each)..."
echo ""

FIXED=0
STILL_CRASHED=0

while IFS= read -r line; do
  sweep=$(echo "$line" | cut -d, -f1)
  label=$(echo "$line" | cut -d, -f2)
  value=$(echo "$line" | cut -d, -f3)

  args=$(build_args "$sweep" "$value")
  if [ -z "$args" ]; then
    echo "WARNING: don't know how to rebuild args for sweep=$sweep value=$value, skipping"
    STILL_CRASHED=$((STILL_CRASHED + 1))
    continue
  fi

  echo "Retrying: $label ($sweep=$value)"

  success=false
  for attempt in $(seq 1 $MAX_RETRIES); do
    output=$($BENCH --reps=$REPS $args 2>&1)
    rc=$?

    if [ $rc -eq 0 ]; then
      # Parse results
      evals=$(echo "$output" | grep "Total evals:" | awk '{print $NF}')
      wall=$(echo "$output"  | grep "Wall time:"   | awk '{print $(NF-1)}')
      eps=$(echo "$output"   | grep "Evals/sec:"   | awk '{print $NF}')
      uspe=$(echo "$output"  | grep "Time per eval:"| awk '{print $(NF-1)}')

      if [ -n "$eps" ]; then
        # Build replacement line
        new_line="$sweep,$label,$value,$evals,$wall,$eps,$uspe"
        # Escape for sed
        old_escaped=$(echo "$line" | sed 's/[&/\]/\\&/g')
        new_escaped=$(echo "$new_line" | sed 's/[&/\]/\\&/g')
        sed -i "s|$old_escaped|$new_escaped|" "$CSVFILE"
        echo "  OK (attempt $attempt): $eps evals/sec, $uspe us/eval"
        success=true
        FIXED=$((FIXED + 1))
        break
      fi
    fi

    echo "  Attempt $attempt failed (exit=$rc), retrying..."
  done

  if [ "$success" = false ]; then
    echo "  STILL CRASHED after $MAX_RETRIES attempts"
    STILL_CRASHED=$((STILL_CRASHED + 1))
  fi

  echo ""
done <<< "$CRASHED_LINES"

echo "========================================"
echo "Retry complete: $FIXED fixed, $STILL_CRASHED still crashed"
echo "Results patched in: $CSVFILE"
