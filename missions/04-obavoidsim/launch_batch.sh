#!/bin/bash

# Amount of obstacles to test with corresponding separation distances
# For <=10 obstacles, sep=10 works fine. For >10, lower sep to fit in same area.
options=("1" "2" "4" "8" "10" "12" "16" "20")
seps=(    "10" "10" "10" "10" "10" "8"  "6"  "5")

# Launch command with each option
for i in "${!options[@]}"; do
    opt="${options[$i]}"
    sep="${seps[$i]}"
    echo "Launching command with: obs=$opt, sep=$sep"
    ./launch_sleep.sh --obs="$opt" --sep="$sep" 10

    tar -czf obavoidsim_obs_"$opt".tar.gz MOOSLog_*/ LOG_*/ XLOG_*/
    # tar -xzf obavoidsim_obs_"$opt".tar.gz  # To extract the tarball if needed
    ./clean.sh
done