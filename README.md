# moos-ivp-thoma904-thesis

|              |                        |
|:------------ |:---------------------- |
| FILE:        | moos-ivp-thoma904-thesis/README |
| DATE:        | 2025/08/11             |
| DESCRIPTION: | Contains beta apps for integrating towed bodies into the MOOS-IVP framework. |


# Introduction

This repository contains codes for extending the MOOS-IVP Autonomy system to factor in towed body behavior. This includes a MOOS application only as of 11AUG25. Constructed to build and launch.


# Applications

The relevant applications for this repo are described below:

| src/             | Description                                 |
|:---------------- |:------------------------------------------- |
| pTowing          | Basic "breadcrumb" model                    |
| pTowing1         | Mass-Spring-Damper model (Recommended)      |
| pTowing2         | Expanded model based on Newman EoM (no work)|

# Missions

| missions/        | Description                                 |
|:---------------- |:------------------------------------------- |
| Towing_1         | Basic alder mission (waypoint and back)     |
| Towing_2         | Leg run mission with complex turns          |

# Behaviors

None as of yet.

# Build Instructions

## Linux and Mac Users

To build on Linux and Apple platforms, execute the build script within this
directory:

```bash
   $ ./build.sh
```

To build without using the supplied script, execute the following commands
within this directory:

```bash
   $ mkdir -p build
   $ cd build
   $ cmake ../
   $ make
   $ cd ..
```

# Launch Instructions

After building navigate to missions/Towing_2 and run the supplied launch script:

'''bash
   $ cd missions/Towing_2
   $ ./launch.sh
'''

# END of README

