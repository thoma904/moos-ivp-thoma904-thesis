#!/bin/bash -e
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: obavoid
#   Author: M.Benjamin
#   LastEd: Oct 2024
#------------------------------------------------------------
#  Part 1: Set convenience functions for producing terminal
#          debugging output, and catching SIGINT (ctrl-c).
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

#------------------------------------------------------------
#  Part 2: Set global variable default values
#------------------------------------------------------------
ME=`basename "$0"`
CMD_ARGS=""
TIME_WARP=1
VERBOSE=""
JUST_MAKE=""
LOG_CLEAN=""
VAMT="1"
MAX_VAMT="1"
RAND_VPOS=""
MAX_SPD="2"

# Monte
XLAUNCHED="no"
NOGUI=""

# Custom
OBSTACLE_AMT=8
SEP=10
ENC=12
MODEL_RADIUS=5
POINTS=""
DRESET=""

#------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                      "
	echo "                                               "
	echo "Options:                                       "
	echo "  --help, -h         Show this help message    " 
	echo "  --verbose, -v      Verbose, confirm launch   "
	echo "  --just_make, -j    Only create targ files    " 
	echo "  --log_clean, -lc   Run clean.sh bef launch   " 
	echo "  --amt=N            Num vehicles to launch    "
	echo "  --rand, -r         Rand vehicle positions    "
	echo "  --max_spd=N        Max helm/sim speed        "
	echo "                                               "
	echo "Options (monte):                               "
	echo "  --xlaunched, -x    Launched by xlaunch       "
	echo "  --nogui, -ng       Headless launch, no gui   "
	echo "                                               "
	echo "Options (custom):                              "
	echo "  --dynamic_reset, -d                          " 
	echo "    Enable dynamice resetting of obstacles in  "
	echo "    the obstacle simulator                     "
	echo "  --obs=<dist> (Default 8)                     " 
	echo "    Number of obstacles                        "
	echo "  --enc=<amt>                                  " 
	echo "    Number of encounters in headless mission   "
	echo "  --sep=<dist>                                 " 
	echo "    Min dist between obstacles (Default 10)    "
	echo "  --points, -p                                 " 
	echo "    Enable the simulated sensor points mode    "
	echo "    in the obstacle simulator                  "
	echo "  --model_radius=<dist>                        "
	echo "    Adjust radius of turn model (Default 5).   "
	exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE=$ARGI
    elif [ "${ARGI}" = "--log_clean" -o "${ARGI}" = "-lc" ]; then
	LOG_CLEAN=$ARGI
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        VAMT="${ARGI#--amt=*}"
	if [ $VAMT -lt 1 -o $VAMT -gt $MAX_VAMT ]; then
	    echo "$ME: Veh amt range: [1, $MAX_VAMT]. Exit Code 2."
	    exit 2
	fi
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RAND_VPOS=$ARGI
    elif [ "${ARGI:0:10}" = "--max_spd=" ]; then
        MAX_SPD="${ARGI#--max_spd=*}"

    elif [ "${ARGI}" = "--xlaunched" -o "${ARGI}" = "-x" ]; then
	XLAUNCHED="yes"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-ng" ]; then
	NOGUI="--nogui"

    elif [ "${ARGI}" = "--dynamic_reset" -o "${ARGI}" = "-d" ]; then
	DRESET=$ARGI
    elif [ "${ARGI:0:6}" = "--obs=" ]; then
        OBSTACLE_AMT="${ARGI#--obs=*}"
    elif [ "${ARGI:0:6}" = "--enc=" ]; then
        ENC="${ARGI#--enc=*}"
    elif [ "${ARGI:0:6}" = "--sep=" ]; then
        SEP="${ARGI#--sep=*}"
    elif [ "${ARGI}" = "--points" -o "${ARGI}" = "-p" ]; then
	POINTS=$ARGI
    elif [ "${ARGI:0:15}" = "--model_radius=" ]; then
        MODEL_RADIUS="${ARGI#--model_radius=*}"

    else 
	echo "$ME: Bad arg:" $ARGI "Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------
#  Part 4: Set starting positions, speeds, vnames, colors
#------------------------------------------------------------
INIT_VARS=" --amt=$VAMT $RAND_VPOS $VERBOSE "
INIT_VARS+=" --obs=$OBSTACLE_AMT --sep=$SEP " #custom
./init_field.sh $INIT_VARS

VEHPOS=(`cat vpositions.txt`)
SPEEDS=(`cat vspeeds.txt`)
VNAMES=(`cat vnames.txt`)
VCOLOR=(`cat vcolors.txt`)

ALL_VNAMES=""

#------------------------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
#------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "============================================"
    echo "  $ME SUMMARY                   (ALL)       "
    echo "============================================"
    echo "CMD_ARGS =      [${CMD_ARGS}]               "
    echo "TIME_WARP =     [${TIME_WARP}]              "
    echo "JUST_MAKE =     [${JUST_MAKE}]              "
    echo "LOG_CLEAN =     [${LOG_CLEAN}]              "
    echo "VAMT =          [${VAMT}]                   "
    echo "MAX_VAMT =      [${MAX_VAMT}]               "
    echo "RAND_VPOS =     [${RAND_VPOS}]              "
    echo "MAX_SPD =       [${MAX_SPD}]                "
    echo "--------------------------------(VProps)----"
    echo "VNAMES =        [${VNAMES[*]}]              "
    echo "VCOLORS =       [${VCOLOR[*]}]              "
    echo "START_POS =     [${VEHPOS[*]}]              "
    echo "--------------------------------(Monte)-----"
    echo "XLAUNCHED =     [${XLAUNCHED}]              "
    echo "NOGUI =         [${NOGUI}]                  "
    echo "--------------------------------(Custom)----"
    echo "DRESET =        [${DRESET}]                 "
    echo "OBSTACLE_AMT =  [${OBSTACLE_AMT}]           "
    echo "ENC =           [${ENC}]                    "
    echo "SEP =           [${SEP}]                    "
    echo "POINTS =        [${POINTS}]                 "
    echo "MODEL_RADIUS =  [${MODEL_RADIUS}]           "
    echo "                                            "
    echo -n "Hit any key to continue launch           "
    read ANSWER
fi

#------------------------------------------------------------
#  Part 6: Launch the Vehicles
#------------------------------------------------------------
VARGS=" --sim --auto --max_spd=$MAX_SPD "
VARGS+=" $TIME_WARP $JUST_MAKE $VERBOSE "
for IX in `seq 1 $VAMT`;
do
    IXX=$(($IX - 1))
    IVARGS="$VARGS --mport=900${IX}  --pshare=920${IX} "
    IVARGS+=" --start_pos=${VEHPOS[$IXX]} "
    IVARGS+=" --stock_spd=${SPEEDS[$IXX]} "
    IVARGS+=" --vname=${VNAMES[$IXX]} "
    IVARGS+=" --color=${VCOLOR[$IXX]} "
# custom
    IVARGS+=" --model_rad=${MODEL_RADIUS} "  
    vecho "Launching vehicle: $IVARGS"

    if [ "${ALL_VNAMES}" != "" ]; then
	ALL_VNAMES+=":"
    fi
    ALL_VNAMES+="$VNAME"
    
    CMD="./launch_vehicle.sh $IVARGS"    
    eval $CMD
    sleep 0.5
done

#------------------------------------------------------------
#  Part 7: Launch the Shoreside mission file
#------------------------------------------------------------
SARGS=" --auto --mport=9000 --pshare=9200 $NOGUI --vnames=$ALL_VNAMES "
SARGS+=" $TIME_WARP $JUST_MAKE $VERBOSE "
# custom
SARGS+=" $POINTS $DRESET "
SARGS+=" --sep=$SEP --enc=$ENC "

vecho "Launching shoreside: $SARGS"
./launch_shoreside.sh $SARGS 

if [ "${JUST_MAKE}" != "" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 8: Unless auto-launched, launch uMAC until mission quit
#------------------------------------------------------------
if [ "${XLAUNCHED}" != "yes" ]; then
    uMAC --paused targ_shoreside.moos
    trap "" SIGINT
    echo; echo "$ME: Halting all apps"
    kill -- -$$
fi

exit 0
