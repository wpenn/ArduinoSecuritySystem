# Arduino Security System - ESE 190 @ UPenn
We built a close-quarters security system for small spaces (e.g. desks, dorm rooms). You can arm and disarm the system from your mobile device. The system begins in the disarmed state and the LED is green. In order to arm the alarm you must type in the set passcode on your mobile device in the Adafruit app (with the number keypad), which appears on the LCD, and press enter (the right arrow). When the alarm is armed, the LED is red and the sensor is searching for motion. If it is detected, the buzzer alarms, the LED flashes red, and the system is waiting for the password to be entered in order to disarm. Additionally, the back arrow allows you to delete your password input and the down arrow allows you to clear your password input.

## General Information

### Team Members
* [Wesley Penn](https://www.linkedin.com/in/wesley-penn) SEAS ‘22
* [Jeremy Kogan](https://www.linkedin.com/in/jeremy-kogan) SEAS ‘24

## Easter Egg
There exists an easter egg in our system. If you guess the correct password and press the up arrow, the Mario theme song will play and you will see the names of our founders on the LCD. 

## Parts List
* Arduino Uno
* Adafruit Bluefruit LE Module
* Liquid Crystal Display (LCD)
* Buzzer
* 220 Ohm resistor
* 100 Ohm Resistor
* 10k Ohm Potentiometer
* PING Ultrasonic Distance Sensor
* LEDs (2x, Blue and Green)
* Wires
* Breadboard

## Design Process & Iterations
We knew from the start that the Bluetooth component would be difficult, finicky, and slow-moving to develop with, so we decided to add that component last. We started with a PIR Motion Sensor, but found that in a couple of painstaking hours that it was very difficult to use. We decided to alternatively use an Ultrasonic sensor to detect distance and assume the setup would be static in closed quarters for success. Once we got that working, we created logic to coordinate the states of the alarm system - disarmed, armed, triggered. We got the Ultrasonic to switch the state from armed to triggered. Then, we added LED actuators to represent the three states, and added the Bluetooth module to control the states. Lastly, as a stretch goal, we added an LCD to show the states, and added a password management system to finish off the project.

## Challenges we ran into
The Bluetooth module is quite inconsistent, requiring us to sometimes factory reset multiple times and/or push down on the device into the breadboard in order for it to work. We also ran into roadblocks trying to get the PIR motion sensor to work before switching to an ultrasonic.

## Future Improvements
* Password management: Password is hardcoded, we want users to be able to set a password on a Bluetooth device
* Additional Hardware: Connect Additional Hardware to alarm. For example, potentially add a lock to a door connected triggered by the alarm, or security lights
