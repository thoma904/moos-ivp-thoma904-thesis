#!/bin/bash
#---------------------------------------------------------
# run_sweep.sh — Parameter sweep for AOF benchmark
#
# Varies one parameter at a time to isolate bottlenecks.
# Output is tab-separated for easy pasting into a spreadsheet.
#---------------------------------------------------------

# Resolve the binary relative to this script's location
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BENCH="$REPO_ROOT/bin/aof_bench"

if [ ! -x "$BENCH" ]; then
  echo "Error: aof_bench not found at $BENCH. Build first."
  exit 1
fi

REPS=50
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTDIR="$REPO_ROOT/src/app_aof_bench/results"
mkdir -p "$OUTDIR"
LOGFILE="$OUTDIR/sweep_${TIMESTAMP}.txt"
CSVFILE="$OUTDIR/sweep_${TIMESTAMP}.csv"

echo "Results will be saved to:"
echo "  Log: $LOGFILE"
echo "  CSV: $CSVFILE"
echo ""

# CSV header
echo "sweep,parameter,value,total_evals,wall_time_s,evals_per_sec,us_per_eval" > "$CSVFILE"

# Helper: run bench, log output, extract CSV row
run() {
  local sweep="$1"
  local label="$2"
  local value="$3"
  shift 3
  echo "--- $label ---" | tee -a "$LOGFILE"
  local output
  output=$($BENCH --reps=$REPS "$@" 2>&1)
  echo "$output" | tee -a "$LOGFILE"
  echo "" | tee -a "$LOGFILE"

  # Parse results from output
  local evals wall eps uspe
  evals=$(echo "$output" | grep "Total evals:" | awk '{print $NF}')
  wall=$(echo "$output"  | grep "Wall time:"   | awk '{print $(NF-1)}')
  eps=$(echo "$output"   | grep "Evals/sec:"   | awk '{print $NF}')
  uspe=$(echo "$output"  | grep "Time per eval:"| awk '{print $(NF-1)}')
  echo "$sweep,$label,$value,$evals,$wall,$eps,$uspe" >> "$CSVFILE"
}

tee_header() {
  echo "$1" | tee -a "$LOGFILE"
}

# ---------------------------------------------------------
# Sweep 1: Simulation horizon (steps = horizon / dt)
# ---------------------------------------------------------
tee_header "===== SWEEP: sim_horizon (dt=0.1) ====="
for hz in 5 10 15 20 25 30 40; do
  run "sim_horizon" "horizon=${hz}s" "$hz" --sim_horizon=$hz --sim_dt=0.1
done

# ---------------------------------------------------------
# Sweep 2: Simulation dt (with fixed horizon)
# ---------------------------------------------------------
tee_header "===== SWEEP: sim_dt (horizon=25) ====="
for dt in 0.5 0.2 0.1 0.05; do
  run "sim_dt" "dt=${dt}s" "$dt" --sim_horizon=25 --sim_dt=$dt
done

# ---------------------------------------------------------
# Sweep 3: Cable length (controls num_nodes)
# ---------------------------------------------------------
tee_header "===== SWEEP: cable_length ====="
for cl in 10 20 30 50 80 100 150; do
  run "cable_length" "cable_len=${cl}m" "$cl" --cable_len=$cl
done

# ---------------------------------------------------------
# Sweep 4: Cable check interval
# ---------------------------------------------------------
tee_header "===== SWEEP: cable_check_interval ====="
for ci in 1 2 3 5 10 20; do
  run "cable_check" "cable_check=$ci" "$ci" --cable_check=$ci
done

# ---------------------------------------------------------
# Sweep 5: Cable start node (skip shallow nodes)
# ---------------------------------------------------------
tee_header "===== SWEEP: cable_start_node ====="
for sn in 0 1 2 3; do
  run "cable_start" "cable_start=$sn" "$sn" --cable_start=$sn
done

# ---------------------------------------------------------
# Sweep 6: Domain resolution
# ---------------------------------------------------------
tee_header "===== SWEEP: domain resolution ====="
for crs in 72 180 360 720; do
  for spd in 11 21 31; do
    run "domain_res" "crs=${crs}_spd=${spd}" "${crs}x${spd}" --crs_pts=$crs --spd_pts=$spd
  done
done

tee_header "===== SWEEP COMPLETE ====="
echo ""
echo "Results saved:"
echo "  Log: $LOGFILE"
echo "  CSV: $CSVFILE"
