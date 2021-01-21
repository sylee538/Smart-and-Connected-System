# Code README

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
