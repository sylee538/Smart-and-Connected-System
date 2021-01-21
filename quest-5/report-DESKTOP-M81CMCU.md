# Secure - Key
Authors: Rene Colato, Edward Hong, SeungYeun Lee

2019-11-22

## Summary
This quest we wired a fob (IR RX/TX circuit) that transmitted data to a hub (same circuit) running on ESPs. The goal is to transmit the Fob_ID and a code to the hub; the hub would then send the its ID along with the Fob_ID and code to the Raspberry Pi. The Raspberry Pi would then consult the database to see if the signal received is valid and, if correct, would push the data to an website which displays a historical list of the username, timestamp, location, and IDs on a list. The Raspberry Pi would host the website and the database of wifi.

## Investigative Question
*Investigative question: comment on the security of your system. How would you best hack into this system if you were so inclined? How could you prevent this attack? Describe the steps.*

### **How to Hack System:**
#### Hardware vulnerabilities:
* A hacker could read the signal from the emitter decode it for themselves and get access to the Fob_ID and code. Both pieces of data are the only credentials required to sign into the system.
#### Software vulnerabilities:
* The system is dependent on a secure wifi connection, hackers could go into insecure wifi networks that hosts the system and extract the data being transferred from the ESP to the RasPi or the node server to the HTML.
* Since the max number of ports is unsigned 16 bits, if a hacker were to populate and overflow the ports with malicious UDP packets the server would crash.
* Since the sign-in data would be posted onto the HTML persistently the data would susceptible to data mining and the login credentials would be exposed. So, if a hacker would be able to get onto the network and access the insecure website all the data would be available to them.

### **How to Prevent Attacks:**
#### Hardware vulnerabilities:
* Secure and monitor the hardware so no malicious people could try to read the signal using a decoder.

#### Software vulnerabilities:
* Ensure that all networks the system connects to are secured and all data transferred are encrypted.
* To prevent UDP overflow, the socket should be bond using the bind function to a port instead of allowing random assignments. Then monitor the port for unusual activity.
* The HTML could have a sign-in process for administrators so that others users wouldn't be able to easily access the data if they made it onto the network.
## Evaluation Criteria
**Criteria Met**
- Database: Our database is hosted on a node server using a Raspberry Pi. It is initially populated with all the fobs logged off. When any fob signs into any hub the IDs of each, the timestamp, location etc. are appended to the database for persistence.
- Queries: The database is queried to show on the console all users that have been signed in and all the related data to those instances of signing in.
- Fobs: The one fob cycles through multiple IDs to simulate multiple fobs. The Fobs communicated with Hubs using and IR bulb to transmit the Fob_ID and a code. When the ID is authenticated the green LED is turned on.
- Hubs: The one hub cycles through multiple IDs to simulate multiple hubs. The hub is connected via wifi to the network. The Hubs receive a Fob_ID and a code and pushes that data along with its own ID to the Raspberry PI via sockets.
- HTML: On button press emits via sockets a call to run the 'Run' program (broken)

**Features with Room for Improvement**
- FOB/HUB Use: The button that cycles the user will only send once. Once the message is sent to node, regardless of success or failure, the user only gets to send until the cycle button gets to them again. An improvement could be done by adding another button to let the user send again.

- Database:

Queries:

- HTML:


## Solution Design
### Roadmap to Completion
**Hardware Setup**
The first iteration of the code is designed so every portion of the code is on an RTOS task, this helps the readability of the code and further addition on the code in the future. The first design originally had a 2-way communication through IR and UDP to decrease latency for the display LED. This is implemented by using a state machine so the sensor would not activate if it was sending data. This prevents any misread data. On the HUB side, the received data would be parsed to the proper format *(fob_id, code, hub_id)* and send it to the Node server through UDP. The FOB would then wait for confirmation that the login was successful and this signal comes from the Node server.

The final design added more utility to the button by making each button click toggle through different user/hub IDs. The two-way IR is removed and all data transfer rely on a 1-way IR and UDP sockets for confirmation signals/data. The delays between each event is also increased so it reduced data traffic and increase LED display for the user to see each stage of the confirmation clearly.

**Node Server Setup**
The node server uses express to host a HTML front-end and has socket connections to it. There is on socket.io channel that sends and receives from the hub ESP. The server also maintains and updates the database for sign in and sign out. The backend also pushes sign-in statements to the HTML via Sockets.

**Client Interface Setup**
The HTML front-end has three buttons that are programmed lock out the user. Each button is clickable and would trigger a script that posts the user log out and timestamp onto the HTML.

## Sketches and Photos

Fob Circuit
![Image](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/blob/master/quest-5/images/Fob_Circuit.jpg)

Hub Circuit
![Image](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/blob/master/quest-5/images/Hub_Circuit.jpg)

Fob State Diagram
![Image](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/blob/master/quest-5/images/fob_state.png)

Hub State Diagram
![Image](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/blob/master/quest-5/images/hub_state.png)

## Supporting Artifacts
- [Link to code](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-5/code)
- [Link to video demo](https://www.youtube.com/watch?v=s3UhZmu-Y_E)

## References

-[Database Brief](http://whizzer.bu.edu/skills/tingodb)

-[MongoDb](https://www.w3schools.com/nodejs/nodejs_mongodb.asp)

-[TingoDB](http://www.tingodb.com/)

-[DB Design Patterns](http://whizzer.bu.edu/briefs/design-patterns/dp-db)

-[IR Brief](http://whizzer.bu.edu/skills/ir-tx-rx)

-[IR Design Patterns](http://whizzer.bu.edu/briefs/design-patterns/dp-irtxrx)

-[Security Brief](http://whizzer.bu.edu/skills/security)

-[State Models Brief](http://whizzer.bu.edu/skills/state-models)

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
