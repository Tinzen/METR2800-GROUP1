# Software-in-the-loop sim

Runs the real `Robot_Controller_V2_1` firmware (step runner, Sequence.h, RobotConfig.h) on your laptop against a simple physics model of the slide (goBILDA 435 rpm motor, friction, gravity on the incline, bucket contact near full extension) and a timing model of the Nano (115200 baud serial, I2C writes).

Use it to check a sequence or timing change before you touch the robot:

```
cd tools/sil_sim
./build_and_run.sh                  # full run with the placeholder slide positions
./build_and_run.sh 14 "" 1650 1740  # try the CAD slide positions (SIM_CONTACT=3150 for the pole)
./build_and_run.sh 18 my.txt        # RUN_CONSOLE with scripted commands
```

Needs g++ (Linux, macOS, or WSL on Windows). The plant numbers in `sim_main.cpp` are estimates. Trust the sequence and timing logic, not the exact slide dynamics. `tools/sequence_editor.html` predicts the same step timing without the plant.
