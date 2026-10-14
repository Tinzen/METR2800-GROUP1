#!/bin/bash
# Software-in-the-loop test: compiles the real firmware for your laptop with
# mock Arduino/servo/encoder headers and a physics model of the slide, runs it,
# and writes a log you can open in tools/run_viewer.html.
# Needs g++ (Linux, macOS, or WSL on Windows).
#   ./build_and_run.sh [seconds] [script.txt] [EXTEND_TICKS]
#   script.txt lines: "<ms> <console command>" (use with RUN_MODE = RUN_CONSOLE)
set -e
cd "$(dirname "$0")"
FW=../../Robot_Controller_V2_1
SECS=${1:-16}; SCRIPT=${2:-}; EXT=${3:-4500}
B=build; rm -rf $B; mkdir -p $B
cp $FW/*.cpp $FW/*.h $B/
# the sim plant needs a realistic extend target; your real value lives in the .ino
sed -E "s/^const float OUTTAKE_EXTEND_TICKS = [0-9.]+f;/const float OUTTAKE_EXTEND_TICKS = ${EXT}.0f;/" $FW/Robot_Controller_V2_1.ino > $B/body.ino
{ echo '#include <Arduino.h>'; cat $B/body.ino; } > $B/sketch.cpp
g++ -std=gnu++11 -O1 -Wall -Wno-unused-function -Imock -I$B -o $B/sim sim_main.cpp $B/sketch.cpp \
    $B/MotorController.cpp $B/MotionProfile.cpp $B/PIDController.cpp $B/ServoAxis.cpp
mkdir -p ../../logs
OUT=../../logs/sim_$(date +%Y%m%d_%H%M%S).csv
{ echo "#CAPTURE,SIMULATED software-in-the-loop run,sim,"; $B/sim $SECS $SCRIPT; } > $OUT
grep -E "^(S|E|R)," $OUT
python3 analyze.py $OUT 2>/dev/null || true
echo "log: $OUT"
