var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var port = process.env.PORT || 8080;
var dgram = require('dgram');

var PORT = 8080;
var HOST = '192.168.1.106';
var control = 0;

var server = dgram.createSocket('udp4');

server.on('listening', function() {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function(message, remote) {
	console.log(remote.address + ':' + remote.port + ' - ' + message);
	server.send(control.toString(), remote.port, remote.address, function(error) {
		if (error)
			console.log('MEH!');
		else
			console.log('OK!');
	});
});

// Bind server to port and IP
server.bind(PORT, HOST);

app.get('/', function(req, res) {
	res.sendFile(__dirname + '/index.html');
});

io.on('connection', function(socket) {
	// socket.emit('data', 'you moderate');

	socket.on('Run', function(msg) {
		control = 1;
		console.log(control);
	});

	socket.on('Stop', function(msg) {
		control = 0;
		console.log(control);
	});

});

http.listen(port, function() {
	console.log('listening on *:' + port);
});