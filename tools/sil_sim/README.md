# Software-in-the-loop sim

Runs the real `Robot_Controller_V2_1` firmware on your laptop against a simple physics model of the slide (goBILDA 435 RPM motor, friction, gravity on the incline, bucket contact near full extension) and a timing model of the Nano (115200 baud serial, 400 kHz I2C).

Use it to check a state-machine or timing change before you touch the robot:

```
cd tools/sil_sim
./build_and_run.sh            # full run, writes logs/sim_<time>.csv
./build_and_run.sh 8 my.txt   # RUN_CONSOLE with scripted commands
```

Needs g++ (Linux, macOS, or WSL on Windows). The plant numbers in `sim_main.cpp` are estimates. Trust the sequence and timing logic, not the exact slide dynamics.
