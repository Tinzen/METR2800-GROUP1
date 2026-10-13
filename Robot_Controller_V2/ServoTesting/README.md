goBILDA Servo Calibration

Servo Positions
Servo	PCA9685 Channel	Minimum (700)	Maximum (2300).

INTAKE
Down: 200
Peak: 350
Scoop: 150
Transfer: 920

SCOOP
Down: 710
Peak: 1000
Flat: 1800
Scoop: 2000
Transfer: 1650

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