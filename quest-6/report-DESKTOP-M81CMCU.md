# Final Rollup
Authors: Rene Colato, Edward Hong, SeungYeun Lee

2019-12-10

## Summary
The flowchart below illustrations the design flow of our final quest. The quest is divided between the ESP32 and the Raspberry Pi; where the ESP32 handles the hardware components of the project and the Raspberry Pi handles the software heavy and user input components of the project.

Focusing on the Raspberry Pi, the Pi will create and maintain a Database (inquiries are being conducted to determine whether LevelDB is a superior alternative to TingoDB given our previous compatibility issues with Sockets.io). The Pi will log split times from the ESP into the Database and push the data to the HTML in a persistent data structure (determination needs to be made between a list and a graph/chart). Furthermore, the Pi will handle processing the video feed from a Logitech camera and display the stream on the HTML. The video stream will go through some video processing to get and decode the QR code. The output of the decoded QR code will then be displayed to the HTML. On the HTML there will be five control buttons for the rover, once the autonomous portion of the race of is over. These buttons will send commands via sockets to the ESP to control the speed and direction of the rover.

The Raspberry Pi serves as the system to interface with the rover, and the ESP serves the purpose of being the brains of the rover itself. The ESP32 will follow the given protocol to run the motor and servo of the Crawler skill to operate the vehicle. Also, the ESP32 will power and monitor the signals from at least five sensors to determine the ranges of obstacles and provide values for the PID to give the autonomous algorithm directives of when to turn where, and stop completely. In addition to that, the ESP32 will receive IR signals to read signals from the checkpoint to stop, slow, or go past in real time. This IR sensor along with a timer will also be the functions that used to determine the split time between checkpoints. Once a split time is calculated, the ESP32 will display the most recent split time to the Alphanumeric Display and, over wifi, send the data to the Raspberry Pi to populate the Database and webpage. Furthermore, in conjunction with the IR and button inputs (from HTML) the PID will modulate between different speeds using a PWM framework.

## Solution Design
### Roadmap to Completion
## Evaluation Criteria
**Criteria Met**
- Web Streaming: Video is streamed via Raspberry Pi using the logitech webcam. The stream is formatted on the Pi and but into the HTML file.
- Front End Web Interface: The interface is coded on the HTML file and is hosted on the Raspberry Pi using a node server. The interface has 5 buttons under the Video feed that is also operated using the Pi.
- Range Sensor: three functioning sensors allow the car to sense distance infront and on the side of the rover, this data can be used to make logic for steering.
- Collision Sensor: An ultrasonic sensor is used to detect distance, the logic will stop the car if its too close to a wall and drive when the closest wall is far enough.
- IR Sensor/Receiver: The IR sensor picks up signal from the beacon and processes the ID and color signal.
- Traffic Signal: The IR sensor uses the IR receiver to read beacon signal, we use the checksum function to make sure the right signal is received.

**Features with Room for Improvement**

**Web Interface**
The current design does have considerations for functional web steering and speed control L,R,F,R, stop. However, given the timetable and the litany of development setbacks the team endured we were unable to develop the software to have messages sent via UDP to execute those instructions. Currently, the code has the buttons and the framework to have UDP programmed, but towards the end of the development process the team determined other factors demanded much more time and effort put into them to maximize our grading potential.

The Web display showing historical splits from database was not implemented and did not receive any development time and thus was not a part of the final design. There were plans to implement the chart, but the difficulties found in quest 5 continued to plague the team.

The web stream was a completely functional component to the project, but there was a significant delay for the stream given the limited power of the Raspberry Pi zero. Given the time, the team would have liked to find other software solutions that yielded better results for streaming in real time and a higher FPS.

The webpage was also successfully able to display all interfaces in the same browser window. But to improve the experience further there would be changes made to the overall look of the UI to be easier on the eyes (bigger buttons), or even some artistic designs implemented so that the page isn't so barebones and rudimentary.

**Devices**
The range sensors in our design are all fully functional and continuously operational. To improve their performance the team would add capacitors to regulate voltage. Another more involved change that would be made would be measuring graphing distance measurements to try to make the two side lidars measure a relatively similar range using an offset function to hopefully improve accuracy and the eventual development of the PID algorithm.

The collision sensor was completely operational and functional, but the most important improvement is to have some form of software that monitors and reduces the amount of noise, spikes passing through. This could be done by having a error checking to make sure two consecutive measurements are realistically close.

**Behaviors**
No Collisions in run:
To ensure that this task is complete the team would implement a full functioning PID, range finding devices would have a functioning offset function, the collision sensor would have a noise, spike reducer, and the web controls would have be completely implemented as well.

Autonomous:
to improve the autonomous performance of our design a fully functioning PID would need to be developed and integrated into all functions of the program.

The QR Decode component to the project was undeveloped because the Raspberry Pi the team used had outdated firmware and was unable to install the correct kernel packages.

LevelDB and TingoDB were both unable to properly operate on the team's Raspberry Pi so the database was not developed. Given the time, the team would have researched alternative database APIs to use that may have yielded better results. Since the database was in-operational the team was also unable to develop the split time function on the Raspberry Pi and thus was unable to print the most recent time to the I2C display.

Ideally if all changes were adequately changed then the rover would have been able to complete the course without nudges or touches.

### Difficulties with QR Implementation
When we were trying to install Zbar in order to decode QR code, we faced difficulties
when installing. It was a problem that we could not fix. I tried looking up solutions; however,
none of them fixed our problem. Unfortunately, we could not even do "./configure", which generates
makefile for the zbar.

**Node Server Setup**
### node.js
The basics of this node server uses dgram, and it listens to the ESP32 for data and
sends information that will get parsed by "udp_client_task()" to drive the crawler.
The data is generated by a front-end html webpage, with 5 buttons. Clicking the buttons
will trigger an emit even through socket to the node server and then to the crawler.

**Client Interface Setup**
### index.html
We were able to display real-time live webcam on our html by connecting html to ip address of the raspberry pi, which the webcam is connected to.

When a stop button on the web is pressed, function stop() sends 0(direciton) to udp client and stops the crawler from moving.

When forward button on the web is pressed, function drive_forward() sends 1(direciton) to udp client and drives the crawler forward.

When left button on the web is pressed, function turn_left() sends 2(direciton) to udp client and turns the crawler to the left.

When right button on the web is pressed, function turn_right() sends 3(direciton) to udp client and turns the crawler to the right.

When backward button on the web is pressed, function drive_backward() sends 4(direciton) to udp client and drives the crawler backwards.

## Sketches and Photos
**FLOW CHART**
![Image](https://github.com/BU-EC444/colato-rene/blob/master/skills/cluster-6-rollup/44-navigation/images/FlowChart.png)


## Supporting Artifacts
- [Link to repo](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-6)
- [Link to video demo](https://www.youtube.com/watch?v=ZgkyNwi-fbQ&fbclid=IwAR1ni6NSB-63wjv2_49J636qlYFxrxvdaZhmuRr93Tjowa1gHlY6ARbH1Sg)


## References
**CRAWLER**
- [Design Patterns](http://whizzer.bu.edu/briefs/design-patterns/dp-esc)

**WEBCAM**
- [Installation guide here](https://pimylifeup.com/raspberry-pi-webcam-server/)

- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/usage/webcams/)

- [Webcam Forums](https://www.raspberrypi.org/forums/viewtopic.php?p=457887#p457887)

- [Instructables Supplemental guide](https://www.instructables.com/id/How-to-Make-Raspberry-Pi-Webcam-Server-and-Stream-/)

**IRCOMM**
- [Design Patterns](http://whizzer.bu.edu/briefs/design-patterns/dp-irtxrx)

- [Sparkfun - Datasheet](https://www.sparkfun.com/products/10266)

- [Sparkfun - Datasheet](https://www.sparkfun.com/products/9349)

- [API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html)

**Database**
- [Design Patterns](http://whizzer.bu.edu/briefs/design-patterns/dp-db)

- [TingoDB Homepage](http://www.tingodb.com/)

- [LevelDB Homepage](https://dbdb.io/db/leveldb)

- [Node + LevelDB](https://www.npmjs.com/package/node-leveldb)

- [Node + MongoDB](https://www.w3schools.com/nodejs/nodejs_mongodb.asp)

**MicroLidar**
- [Sparkfun - Datasheet](https://www.sparkfun.com/products/14588)

- [Sparkfun - Datasheet](https://cdn.sparkfun.com/assets/5/e/4/7/b/benewake-tfmini-datasheet.pdf)

- [API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/uart.html)

- [API Reference Examples](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart)

**Lidar**
- [Rangefinder Datasheet](https://www.robotshop.com/community/blog/show/lidar-lite-laser-rangefinder-simple-arduino-sketch-of-a-180-degree-radar)

- [Github Reference](https://github.com/PulsedLight3D)

- [Operations Manual](https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf)

**PID**
- [PID](http://whizzer.bu.edu/briefs/design-patterns/dp-pid)

**Wheel Speed**
- [datasheet](https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide#example-circuit)

- [API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/pcnt.html)

- [Timer API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/timer.html)

**UltraSonic II**
- [Sparkfun Page](https://www.sparkfun.com/products/15569)

- [Datasheet](https://cdn.sparkfun.com/assets/b/3/0/b/a/DGCH-RED_datasheet.pdf)

**Node**
- [Design Patterns](http://whizzer.bu.edu/briefs/design-patterns/dp-nodejs)

- [Node Homepage](https://nodejs.org/en/)

- [Node Tutorial](https://www.w3schools.com/nodejs/default.asp)

- [Serial port Reference](https://www.npmjs.com/package/serialport)

**Alphanumeric Display**
- [Brief](http://whizzer.bu.edu/briefs/alphanumeric)

- [Code Examples](https://github.com/BU-EC444/code-examples/tree/master/i2c-display)

- [Display Reference](https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing/usage#library-reference-4-14)

- [Display Code Reference](https://github.com/adafruit/Adafruit_LED_Backpack/blob/master/Adafruit_LEDBackpack.cpp)

- [Hardware Reference](https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing)

- [Hardware Reference](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf)

- [Hardware Reference](https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing?view=all)

- [i2c Tutorial](https://learn.sparkfun.com/tutorials/i2c)

- [i2c Tutorial](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/i2c.html)

- [i2c Tutorial](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c)

- [ASCII Reference](https://en.wikipedia.org/wiki/ASCII)

**Previous Quests for Reference**
- [Link to Quest1](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-1)

- [Link to Quest2](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/rene-retro/quest-2/code)

- [Link to Quest3](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-3/update)

- [Link to Quest4](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-4/code)

- [Link to Quest5](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-5/code)
-----

## Reminders

- Video recording in landscape not to exceed 270s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
