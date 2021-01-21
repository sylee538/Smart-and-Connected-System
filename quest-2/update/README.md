# Code README
Authors: SeungYeun Lee, Rene Colato, Edward Hong

*Updated on 11/24/2019*

**Evaluation Criteria:**
- Sensor Data: Ultrasonic, IR and temperature data is sampled using ADC, data gets concatenated into a print statement and prints in serial. Data is taken every 2 second.
- Node Server: Filestream opened to serial port so the print statement is read, **scanf()** is used to get the individual datas and all pushed into individual arrays. Every time there is new data, an internal timer increments to create time for each data point. Data is sent every 2 second.
- Socket: Arrays for each individual data gets emitted by Socket.IO and onto HTML, arrays are then pushed into table that CanvasJS graphs from. Once pushed, graph is re-rendered and updated.
- Realtime: The data from the sensor is sent continuously and plotted on HTML with a slight delay. Delay comes from Wifi latency and background processes.

**Solution Design:**
**Code Setup**
      The code is arranged so every feature is an RTOS non-blocking task, this helps the readability of the code and further addition on the code in the future. The code is mostly a combination of all the skills that were involved and properly arranged. 4 type of data is being generated: battery, distance(ultrasonic), distance(infrared), temperature. All the data are generated in one task, the ADC for each sensor is sampled together and several functions convert ADC reading to more readable data (eg. mV to C/cm). Once ADC is converted, a print statement in **app_main()** prints the result with **printf()**. The data from the sensor are all in global variables.

**Node Server Setup**
      Node server is based around a file stream that listens into the serial port. The print statement is parsed with **scanf()** and saved into an array. Error check is done on the array to ensure the node server doesn't emit empty data points to the HTML. The data from the sensor is concatenated with an internal timer that increments every 2 second when there is new data.

**Client Interface Setup**
      The client interface has a graph made from CanvasJS. 4 Arrays are initialized to prepare data for the graph to use, these 4 correspond to the 4 types of data that is coming from the ESP32 chip. Once this is set, they are added to the 'data' field in "chartContainer" so they can be used to plot as the array gets populated with data. The data is from Socket.IO, where the data sent from the Node Server gets received here.


**Updated Demo Link:**
    [Link to video demo](https://www.youtube.com/watch?v=YbRitGiIt1I&feature=youtu.be)

**Updated Code**
      [Link to code](https://github.com/BU-EC444/Team1-Lee-Hong-Colato/tree/master/quest-2/update)


## Serial.JS
This JS is responsible to listen to the console response from the ESP32 unit. The ESP32 prints out data results in CSV format, this JS uses a file stream to put the result into 'data.csv' file. Before putting the result into the file, the lines are first parsed properly before appending into the file.

## Index.JS
This JS is responsible for translating the CSV from 'serial.js' into JSON filetype for easier use. This JS uses a file stream and Papa Parse, an external library, to recognize CSV syntax and convert to JSON. Once converted, it initializes the HTTP server to port 3000.

Socket.IO does not serve any purpose here.

## Index.HTML
The HTML file is mostly comprised of scripts. The first script is responsible for graphing, the code is sourced from CanvasJS website. Here, 4 arrays are initialized to prepare data for the graph to use, these 4 correspond to the 4 types of data that is coming from the ESP32 chip. Once this is set, they are added to the 'data' field in "chartContainer" so they can be used to plot as the array gets populated with data. Then time is set so the graph has a sense of time, this helps the refresh rate of the graph.

In order for the graph to get data, the AJAX method from jQuery is used, where it retrieves 'data.JSON' from the file directory. In the JSON file, the data is formatted so it gives "Source Timestamp Value". So depending on the source a switchcase will push values to its corresponding array that was initialized in the beginning. Then the legend is updated on the graph and it is called so it updates.

In the body, it includes code for Socket.IO to work with its counterpart in 'index.JS'. The code serves no functional purpose.

## Data.CSV
This file stores the CSV file read from the serial port from ESP32 chip. The first line is always the header.

## Data.JSON
This file stores the JSON data that was translated from 'data.csv'. This is called by 'index.HTML' to call the individual values that the Canvas use to graph.
