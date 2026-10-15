goBILDA Servo Calibration

Servo Positions
Servo	PCA9685 Channel	Minimum (700)	Maximum (2300).

// Arm Positions
const int A_START = 940;
const int A_DOWN = 200;
const int A_PEAK = 350;
const int A_SCOOP = 150;
const int A_TRANSFER = 900;
const int A_MOVE = 500;

// Scoop Positions
const int S_START = 2300;
const int S_DOWN = 700;
const int S_PEAK = 1100;
const int S_SCOOP = 1900;
const int S_PICKUP = 2100;
const int S_TRANSFER = 1800;


also i want the intake to gradually go from 200 to 350 to 150 as the scoop goes from 710 to 1800.


Overview

This program calibrates three goBILDA servos using an Adafruit PCA9685 PWM driver.

It allows you to:
- Select and control individual servos.
- Set pulse widths in microseconds.
- Adjust positions incrementally.
- Identify safe operating limits.

Hardware: Arduino-compatible board, PCA9685, goBILDA servos
PWM frequency: 50 Hz
Serial baud rate: 115200

Getting Started
1. Upload the calibration sketch.
2. Open Tools → Serial Monitor in the Arduino IDE.
3. Set the following:
   - Baud rate: 115200
   - Line ending: Newline
Note: No servo movement is commanded automatically at startup.

Command	Action
s 4	Select Intake Right
s 5	Select Intake Left
s 3	Select Intake Scoop
p 1500	Set pulse width to 1500 µs
+	Increase pulse width by one step
-	Decrease pulse width by one step
step 10	Set adjustment step to 10 µs
status	Display current settings
?	Display help

Selecting a servo does not move it. Set an initial position using p before using + or -.