const fs = require('fs');
const papa = require('papaparse');
const file = fs.createReadStream('data.csv');
var app = require('express')();
var http = require('http').createServer(app);
var io = require('socket.io')(http);

// Initializes table for data
var myRows = {};

// Papa Parse parses CSV data into JSON
papa.parse(file, {
	download: true,
	header: true,
	dynamicTyping: true,
	complete: function(result) { // Runs once completed
		//  Write data into data.json
		console.log(result.data);
		fs.writeFile("./data.json", JSON.stringify(result.data), (err) => {
			if (err) {
				console.error(err);
				return;
			};
			console.log("File has been created");
		});
	}
});


app.get('/', function(req, res) {
	res.sendFile(__dirname + '/index.html');
});

// Socket.io
io.on('connection', function(socket) {
	socket.emit('news', {
		hello: 'world'
	});
	socket.on('my other event', function(data) {
		console.log(data);
	});
});

http.listen(3000, function() {
	console.log('listening on *:3000');
});