var app = require('express')();
const fs = require('fs');
var http = require('http').Server(app);
var io = require('socket.io')(http);
var port = process.env.PORT || 3000;

// Empties data
fs.writeFile('/path/to/file', '', function() {
	console.log('emptied')
})

app.get('/', function(req, res) {
	res.sendFile(__dirname + '/index.html');
});

var result_data = [];
var dataA = {},
	dataB = {};

// initial value
var yValue1 = 600;
var yValue2 = 605;

var time = new Date;
// starting at 9.30 am
time.setHours(9);
time.setMinutes(30);
time.setSeconds(00);
time.setMilliseconds(00);

function updateChart(count) {
	count = count || 1;
	var deltaY1, deltaY2;
	for (var i = 0; i < count; i++) {
		time.setTime(time.getTime() + 2000);
		deltaY1 = .5 + Math.random() * (-.5 - .5);
		deltaY2 = .5 + Math.random() * (-.5 - .5);

		// adding random value and rounding it to two digits.
		yValue1 = Math.round((yValue1 + deltaY1) * 100) / 100;
		yValue2 = Math.round((yValue2 + deltaY2) * 100) / 100;

		dataA = {
			x: time.getTime(),
			y: yValue1
		};
		dataB = {
			x: time.getTime(),
			y: yValue2
		};
		// pushing the new values
		result_data.push({
			Source: 'dataA',
			x: dataA.x,
			y: yValue1
		}, {
			Source: 'dataB',
			x: dataA.x,
			y: yValue2
		});
	}
}
updateChart(100);
setInterval(function() {
	updateChart();
	fs.writeFile("./data.json", JSON.stringify(result_data), (err) => {
		if (err) {
			console.error(err);
			return;
		};
	});
}, 2000);

io.on('connection', function(socket) {
	setInterval(function() {
		socket.emit('data_a', dataA);
		socket.emit('data_b', dataB);
	}, 1000);
});


http.listen(port, function() {
	console.log('listening on *:' + port);
});