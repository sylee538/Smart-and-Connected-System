# Code Readme
## ESP Q3 (quest3.c)
*All component in the code are parallel task, each function will be the header in the following section.*

### task_accel()
This function uses read16 to load X, Y, Z and prints them to the console

### calcRP()
takes in the x, y, and z values and calculates the roll and pitch of the accelerometer. Then it prints those values - intended to be used for debugging. Then it goes through a sequence of conditional statements that are figuring out if the accelerometer is tilted to a pitch and z angle that are conducive to a person in stride and increments a step flag if that is the case. This flag is passed to the task_adx1343 function and adds to the overall step counter- indicating how many steps are taken.

### task_UDPcomm()
This task is responsible for establishing a UDP socket with the Node server. To send data, the code first creates a socket to the port-forwarded node server. It then sends a desired payload, in this case, its the data that we get from the various peripherals implemented in the code.
The task also recieves instructions from the server, which is done by receiving a string that gets parsed to get control fields and values. These fields & values get sent to "control()" to determine what action to take.

### control()
Control is just a switch case to determine what to do with the various peripherals. The switch case can change timers and flags to turn on or off certain parts. For instance, for code 1 the code will change the water timer to the value received.

### task_time()
This task runs a time ticker that ticks every second, each tick increments a "second" variable, which is mainly used to flicker LED every second. This task also sets a timer for the water LED to trigger.

### task_temperature()
This function takes the ADC reading of a voltage divider with a 15k resistor and the 10k resistance thermistor to find the voltage across the thermistor using Ohm's Law. Using the datasheet, the voltage is then translated to temperature.

### task_alert()
This task turns on the Alert LED to help the user find the peripheral. The code lets the LED flicker every second until the user triggers the vibration switch. Once this is done, the "look_flag" will stop the flickering and turn off.

### task_vib()
This task listens to the vibration switch and triggers a hardware interrupt. The vibration switch behaves like an onboard button, and to debounce, it uses the following logic are all true:
  1. If number of interrupts > 1
  2. Not edge of signal (same state)
  3. Time from last activation > 10ms
If true, the code will change the onboard LED to show the user an interrupt has been detected and changes the "look_flag" to change the behavior of tasks.

### task_waterTimer()
The task turns on the water LED that tells the user to drink water. Most of the logic is in "task_time()", where after **X** seconds it triggers a "water_flag". Once the "water_flag" is true, it starts flickering until the user touches the vibration switch and resets with "look_flag".

### task_battery()
The task changes an LED to display the battery level based on the variable "battery". It uses PWM to change the intensity of the LED to represent how full the battery is. The duty intensity is decided by using a logarithmic scale.

## Node Q3
*All packages are in node_modules*
### index.js
The basis of this node server uses dgram, and it listens to the ESP32 for data and sends information that will get parsed by "task_control()" to do certain things.
The data received from the ESP32 should be passed through socket.IO, though it was implemented but since it doesn't function, it is left out

### index.html
This is the HTML with CanvasJS setup, it has sockets implemented but it doesnt work.

### data.JSON
This is where the data of the ESP gets stored
