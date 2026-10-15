#!/bin/bash
# Software-in-the-loop test: compiles the real firmware for your laptop with
# mock Arduino / I2C / encoder headers and a physics model of the slide, runs
# it, and writes a log you can open in tools/run_viewer.html.
# Needs g++ (Linux, macOS, or WSL on Windows).
#   ./build_and_run.sh [seconds] [script.txt] [X_CREEP_mm X_TIP_mm]
#   script.txt lines: "<ms> <console command>" (use with RUN_MODE = RUN_CONSOLE)
#   SIM_CONTACT=<counts> sets where the bucket meets the pole in the plant.
set -e
cd "$(dirname "$0")"
FW=../../Robot_Controller_V2_1
SECS=${1:-16}; SCRIPT=${2:-}; XC=${3:-}; XT=${4:-}
B=build; rm -rf $B; mkdir -p $B
cp $FW/*.cpp $FW/*.h $B/
# optional: try the real (CAD) slide positions instead of the safe placeholders
[ -n "$XC" ] && sed -i -E "s/^const float X_CREEP +=.*;/const float X_CREEP = ${XC}.0f;/" $B/Sequence.h
[ -n "$XT" ] && sed -i -E "s/^const float X_TIP +=.*;/const float X_TIP = ${XT}.0f;/" $B/Sequence.h
{ echo '#include <Arduino.h>'; cat $FW/Robot_Controller_V2_1.ino; } > $B/sketch.cpp
g++ -std=gnu++11 -O1 -Wall -Wno-unused-function -Wno-narrowing -Imock -I$B -o $B/sim sim_main.cpp $B/sketch.cpp \
    $B/MotorController.cpp $B/MotionProfile.cpp $B/PIDController.cpp $B/ServoAxis.cpp
mkdir -p ../../logs
OUT=../../logs/sim_$(date +%Y%m%d_%H%M%S).csv
{ echo "#CAPTURE,SIMULATED software-in-the-loop run,sim,"; $B/sim $SECS $SCRIPT; } > $OUT
grep -E "^(S|E|R)," $OUT
echo "log: $OUT"
