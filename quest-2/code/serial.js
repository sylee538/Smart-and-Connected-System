//source: https://medium.com/@machadogj/arduino-and-node-js-via-serial-port-bcf9691fab6a
const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');

var fs = require('fs');

fs.appendFile('data.csv', 'Source,Time,Value\n', function (err) {
  if (err) throw err;
  console.log('Updated!');
});

const port = new SerialPort('COM3', { baudRate: 115200 });
const parser = port.pipe(new Readline({ delimiter: '\n' }));
// Read the port data
port.on("open", () => {
  console.log('serial port open');
});
parser.on('data', data =>{
  console.log('Reading from esp32:', data);
  fs.appendFile('data.csv', data+'\n', function (err) {
    if (err) throw err;
    console.log('Updated!');
  });
});
