var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var port = process.env.PORT || 3000;
var sscanf = require('sscanf');

const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const sp = new SerialPort('/dev/cu.SLAB_USBtoUART', {
	baudRate: 115200
});
const parser = sp.pipe(new Readline({
	delimiter: '\n'
}));

sp.on("open", () => {
	console.log('serial port open');
});

var sensorD = {};
parser.on('data', data => {
	sensorD = sscanf(data, "%f%f%f%f")
});

var result_data = [];
var data_V = {},
	data_IR = {},
	data_UT = {},
	data_C = {};



var time = new Date;
var flag = 1;
// starting at 9.30 am
time.setHours(0);
time.setMinutes(0);
time.setSeconds(00);
time.setMilliseconds(00);



io.on('connection', function(socket) {
	setInterval(function() {
		updateChart(sensorD)
	}, 2000);

	function updateChart(data) {
		console.log(data);
		if ((data[0] != NaN || data[1] != NaN || data[2] != NaN || data[3] != NaN) && data.length == 4) {
			console.log("sent");
			data_V = {
				x: time.getTime(),
				y: data[0]
			};
			data_IR = {
				x: time.getTime(),
				y: data[1]
			};
			data_UT = {
				x: time.getTime(),
				y: data[2]
			};
			data_C = {
				x: time.getTime(),
				y: data[3]
			};
			time.setTime(time.getTime() + 2000);
			socket.emit('data_V', data_V);
			socket.emit('data_IR', data_IR);
			socket.emit('data_UT', data_UT);
			socket.emit('data_C', data_C);
		}
	}
});

http.listen(port, function() {
	console.log('listening on *:' + port);
});

app.get('/', function(req, res) {
	res.sendFile(__dirname + '/index.html');
});