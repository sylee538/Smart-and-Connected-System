var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var port = process.env.PORT || 8080;
var dgram = require('dgram');
var server = dgram.createSocket('udp4');


var PORT = 8080;
var HOST = '192.168.1.118';
var REMOTE = '192.168.1.104'; //esp ip static address

var direction = 0;

server.on('listening', function() {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function(message, remote) {
	console.log(remote.address + ':' + remote.port + ' - ' + message);
	server.send(direction.toString(), remote.port, remote.address, function(error) {
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

	socket.on('drive', function(direction) {
    if(direction == 0) {
      console.log("stop");
			server.send(direction, PORT, REMOTE);
    }
    else if(direction == 1){
      console.log("forward");
			server.send(direction, PORT, REMOTE);
    }
    else if(direction == 2){
      console.log("left");
			server.send(direction, PORT, REMOTE);
    }
    else if(direction == 3){
      console.log("right");
			server.send(direction, PORT, REMOTE);
    }
    else if(direction == 4) {
      console.log("backward");
			server.send(direction, PORT, REMOTE);
    }
});
});

http.listen(port, function() {
	console.log('listening on *:' + port);
});
