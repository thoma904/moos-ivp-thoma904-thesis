# moos-ivp-thoma904-thesis

|              |                                                                                                                                                                                                         |
| :----------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| FILE:        | moos-ivp-thoma904-thesis/README                                                                                                                                                                         |
| DATE:        | 2026/04/28                                                                                                                                                                                              |
| DESCRIPTION: | MOOS-IvP applications and IvP behaviors that extend the autonomy system to account for towed-body dynamics, including obstacle avoidance and turn management for a vehicle towing a cable-coupled body. |

# Introduction

This repository extends the MOOS-IvP autonomy system to account for a towed body whe avoiding obstacles. It provides a set of MOOS applications that simulate the towed body
and cable, IvP behaviors that bias the helm's decisions to keep the towed body
safe, and a progression of missions used to develop and validate the approach.

# Directory Structure

| Path      | Description                            |
| :-------- | :------------------------------------- |
| lib/      | Generated shared libraries (behaviors) |
| missions/ | MOOS missions and simulation scenarios |
| scripts/  | Helper scripts                         |
| src/      | Source for applications and behaviors  |

# Applications

| src/               | Description                                                       |
| :----------------- | :---------------------------------------------------------------- |
| pTowing            | Mass-spring-damper towed-body simulator (current model)           |
| pCable             | Cable model coupling tow vehicle and towed body                   |
| pTowObstacleMgr    | Obstacle manager extended to account for towed body and cable     |
| pTowTurnMgr        | Manages turn execution while towing (e.g. Williamson-style turns) |
| uFldTowObstacleSim | Shoreside/field-level obstacle simulator for towed-body missions  |
| app_aof_bench      | AOF benchmarking utility                                          |
| Archive/           | Earlier prototypes (breadcrumb, Newman EoM attempt, area, circle) |

# Behaviors

Built into `lib/` from [src/lib_behaviors-test/](src/lib_behaviors-test/):

| Behavior             | Description                                                  |
| :------------------- | :----------------------------------------------------------- |
| BHV_Towing           | Core towed-body coupling behavior                            |
| BHV_TowedTurn        | Turn-management behavior aware of towed body                 |
| BHV_TowObstacleAvoid | Obstacle avoidance that protects both vehicle and towed body |
| BHV_TowSafety        | Safety constraints on tow geometry                           |
| BHV_Williamson       | Williamson turn behavior                                     |
| BHV_SimpleWaypoint   | Minimal waypoint behavior used for testbed missions          |

# Missions

| missions/         | Description                                              |
| :---------------- | :------------------------------------------------------- |
| 01-Minehunting    | Minehunting mission with towed sensor                    |
| 02-obavoidfull    | Full obstacle-avoidance mission with towed body          |
| Simulations/04-23 | Iterative simulation series used to tune and validate    |
|                   | obstacle avoidance, cable modeling, and turn management. |
|                   | Numbering reflects development order; later folders      |
|                   | (e.g. 14-bestconfig, 21–22-warp/horizon variants,       |
|                   | 23-increasedreflector_1mpad) hold the validated configs. |
| Archived_missions | Earlier mission iterations kept for reference            |

See [missions/Simulations/](missions/Simulations/) for the full list.

# Build Instructions

## Linux and Mac

To build using the supplied script:

```bash
   $ ./build.sh
```

To build manually:

```bash
   $ mkdir -p build
   $ cd build
   $ cmake ../
   $ make
   $ cd ..
```

To clean:

```bash
   $ ./clean.sh
```

# Environment Variables

For the helm to find the IvP behaviors built into `lib/`, add this repo's
`lib/` directory to `IVP_BEHAVIOR_DIRS`, and add `bin/` to your `PATH` so
pAntler can launch the applications.

# Launch Instructions

After building, navigate to a mission directory and run its launch script.
For example, to run the full obstacle-avoidance mission:

```bash
   $ cd missions/02-obavoidfull
   $ ./launch.sh {time warp if desired}
```

Most simulation folders also include `launch_batch.sh` for repeated runs and
`clean.sh` to clear logs between launches.

# END of README
