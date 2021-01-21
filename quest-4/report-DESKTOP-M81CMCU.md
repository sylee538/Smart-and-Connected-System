# Sensor Central
Authors: SeungYeun Lee, Rene Colato, Edward Hong

2019-11-14

## Summary
This quest we wired a crawler to drive without collisions between two endpoints on a road segment, the goal is to wire the rover to become more autonomous. This is achieved by having multiple sensors wired around the rover to determine its distance around the rover to prevent it from crashing. We also have a functional ESP32 that communicates with a node server through a two-way UDP socket. The ESP32 request for control instruction from the node server and uses the received instruction to control the rover.

## Investigative Question
*Investigative question: Define what you would want in a better sensor for the vehicle. Be very specific in quantifying it’s performance. Please cite materials found on the web to support your response.*

The goal of this quest is to have a crawler drive properly in a controlled environment, such as a straight and narrow corridor. So in this case, a good sensor could be a sensor with low power draw and can have low accuracy for such a controlled enviornment.

However, if the rover was to be tested in a more uncontrolled environment, a better sensor would demand more. For instance, it would need higher precision in the reading that it provides and needs higher reliability. Reliability in the sense that the sensor's performance isn't impeded by the environment. For instance the LIDAR would be terrible at perceiving distance in a room made of glass since glass is a terrible medium at reflecting perceivable light for the sensor.

## Evaluation Criteria
**Criteria Met**
- Displays speed: An IR sensor is hooked on an alternating pattern of black and white to create pulses that translate to rotational speed and then to m/s. This is then displayed on the ADAFRUIT display.
- Wireless Control: A front-end access with buttons on the HTML emits events to the node server, which is then sent through a UDP socket back to the rover to start or stop the rover.
- Collision Prevention: using a LIDAR, the rover stops before the distance is at a certain value **X** to prevent collision. The value **X** is hardcoded to be the displacement from the LIDAR to the tip of the car.
- Steering: The MicroLIDAR steers the car away from the nearest wall and maintains a straight line in the center of the hallway. The steering is determined by the difference of distance of each LIDAR from the wall.
- Wheel-speed: the speed of the car is maintained roughly at 0.15 m/s, the speed is determined by trial and error to achieve this speed.

**Features with Room for Improvement**
The PID features in this project can use some improvement, both the steering and wheel-speed needed some form of hard-coded value to properly calibrate it to the required values. Properly implementing a PID can make the rover more reliable. Furthermore, the collision detection is done with a LIDAR which is not as reliable as an ultrasonic sensor, since the ultrasonic sensor would remain reliable despite the opacity of the wall in front of the rover.

## Solution Design
### Roadmap to Completion

**Code Setup**

The code is arranged so every feature is an RTOS non-blocking task, this helps the readability of the code and further addition on the code in the future. This coding style has worked for the previous quests but has stumped our progress for a quest of this size. Having 5 different sensors in 5 different tasks has created more unforseen problems. Even though they are on the same priority level and executing in parallel, the time interval between each sensor's reading is too long so one sensor would get delayed.

So instead, the RTOS is used more in moderation in this quest, where sensors are grouped by direction, so all sensors on the left would be in the same task and so on. This solved many issues with reading data.

The code also uses past skills like UDP and Node to setup a front-end control for the rover.

**Node Server Setup**

The node server uses express to host a HTML front-end and has socket connections to it. There are two socket.io channels and depending on which gets triggered, it changes a global variable "ins" to 0 or 1. This then gets sent back to the rover through UDP, which is implemented with the "dgram" package.

**Client Interface Setup**

The HTML front-end has a "Run" and "Stop" button to control the rover. Each button is clickable and would trigger a button event that emits a 1 or 0 for the latter. These emit event gets sent back to the node server through Socket.IO.


## Sketches and Photos
Rover Setup:
<center><img src="./images/rover_setup.png" width="70%" /></center>  

Front End:
<center><img src="./images/front_end.png" width="70%" /></center>  

Design Flow:
<center><img src="./images/design.png" width="70%" /></center>  


## Supporting Artifacts
- [Link to code](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-4/code)
- [Demo](https://www.youtube.com/watch?v=dKg38KCROLA&feature=youtu.be)

## References
- [Socket.io Get Started](https://socket.io/get-started/chat/)
- [Pulse Counter](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/pcnt.html)
- [Timer](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/timer.html)
- [PID](https://en.wikipedia.org/wiki/PID_controller)
- [LIDAR](http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf)
- [MicroLIDAR](https://cdn.sparkfun.com/assets/5/e/4/7/b/benewake-tfmini-datasheet.pdf)
- [ESC Calibration](http://whizzer.bu.edu/briefs/design-patterns/dp-esc)

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
