var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var port = process.env.PORT || 8080;
var dgram = require('dgram');
var sscanf = require('sscanf');

var PORT = 8080;
var HOST = '192.168.1.106';
var control = 0;
var send = 0;

var result_data = [];
var data_STEP = {},
	data_C = {},
	data_BAT = {};
var sensorD = {};
var time = new Date;
time.setHours(0);
time.setMinutes(0);
time.setSeconds(00);
time.setMilliseconds(00);
var server = dgram.createSocket('udp4');

// function updateChart(data) {
// 	if (data[0] != NaN || data[1] != NaN || data[2] != NaN) {
// 		data_STEP = {
// 			x: time.getTime(),
// 			y: data[0]
// 		};
// 		data_C = {
// 			x: time.getTime(),
// 			y: data[1]
// 		};
// 		data_BAT = {
// 			x: time.getTime(),
// 			y: data[2]
// 		};
// 	}
// 	time.setTime(time.getTime() + 1000);
// }

server.on('listening', function() {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function(message, remote) {

	console.log(remote.address + ':' + remote.port + ' - ' + message);
	sensorD = sscanf(message.toString(), "%f%f%f");
	console.log(sensorD);
	server.send("Ins " + control.toString(), remote.port, remote.address, function(error) {
		if (error)
			console.log('MEH!');
		else {
			console.log('OK!');
			if (control == 3) {
				control = 0;
			}
		}
	});
});

server.bind(PORT, HOST);

app.get('/', function(req, res) {
	res.sendFile(__dirname + '/index.html');
});

io.on('connection', function(socket) {
	console.log("Connected")
	setInterval(function() {
		updateChart(sensorD)
	}, 1000);

	function updateChart(data) {
		if ((data[0] != NaN || data[1] != NaN || data[2] != NaN) && data.length == 3) {
			data_STEP = {
				x: time.getTime(),
				y: data[0]
			};
			data_C = {
				x: time.getTime(),
				y: data[1]
			};
			data_BAT = {
				x: time.getTime(),
				y: data[2]
			};
			time.setTime(time.getTime() + 1000);
			socket.emit('data_STEP', data_STEP);
			socket.emit('data_C', data_C);
			socket.emit('data_BAT', data_BAT);
		}
	}
	// Bind server to port and IP

	socket.on('W_R', function(msg) {
		control = 1;
		console.log("WATER");
		console.log(control);
	});

	socket.on('W_S', function(msg) {
		control = 2;
		console.log("WATER");
		console.log(control);
	});

	socket.on('A_R', function(msg) {
		control = 3;
		console.log("ALERT");
		console.log(control);
	});
	//
	// socket.on('A_S', function(msg) {
	// 	control = 4;
	// 	console.log("ALERT");
	// 	console.log(control);
	// });

});

http.listen(port, function() {
	console.log('listening on *:' + port);
});