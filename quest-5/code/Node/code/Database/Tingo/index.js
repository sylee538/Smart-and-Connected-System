var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var port = process.env.PORT || 8080;
var dgram = require('dgram');
var Db = require('tingodb')().Db;
assert = require('assert');
fs = require('fs');

var PORT = 8080;
var HOST = '192.168.1.117';
var REMOTE = '192.168.1.104';
var control;
var fob_msg = "";

var server = dgram.createSocket('udp4');

server.on('listening', function() {
  var address = server.address();
  console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function(message, remote) {
  console.log(remote.address + ':' + remote.port + ' - ' + message);
  fob_msg = message.toString();
  console.log("HERE:")
  console.log(fob_msg);
  updateData(fob_msg);
  updatePresence();
  if (message == "STRING")
    control = "s";
  else {
    control = "f";
  }
  server.send(control, PORT, REMOTE)
});

app.get('/', function(req, res) {
  res.sendFile(__dirname + '/index.html');
});

// Bind server to port and IP

io.on('connection', function(socket) {
  // socket.emit('data', 'you moderate');
console.log("COnnected");
  socket.on('Run', function(msg) {
    console.log(msg);
    console.log("msg");
    // updatePresence();
  });

});

var db = new Db('./', {});
// Fetch a collection to insert document into
var collection = db.collection("newData.json");

// Insert a single document
fs.readFile('data2.txt', (err, data) => {
  if (err) throw err;
  var stringData = data.toString();
  // stringData = stringData
  var list = stringData.split("\n");
  // console.log(list)
  list.shift();
  list.pop(-1)
  for (var i = 0; i < list.length; i++) {
    list[i] = list[i].replace("\r", "");
    list[i] = list[i].replace(" ", "\t");
    list[i] = list[i].split("\t");
    list[i] = {
      "fob_ID": list[i][0],
      "hub_ID": list[i][1],
      "Person": list[i][2],
      "Time": list[i][3],
      "Location": list[i][4],
      "Presence": list[i][5]
    }
    collection.insert(list[i])
  }

  // collection.find({
  //   hub_ID: "1"
  // }).toArray(function(err, results) {
  //   if (err) {
  //     console.error(err);
  //   } else {
  //     console.log(results);
  //     //  response.send("192.unique.host/bleh", results)
  //   }
  // });
  // collection.find({
  //   Presence: "1"
  // }).toArray(function(err, results) {
  //   if (err) {
  //     console.error(err);
  //   } else {
  //     console.log(results);
  //     //  response.send("192.unique.host/bleh", results)
  //   }
  // });

  // if (err) {
  //   console.error(err);
  // } else {
  //   //  response.send("192.unique.host/bleh", results)
  // }
});

var updateData = function(input_str) {
  var testString = fob_msg;
  var newList = testString.split(' ');
  // console.log(newList);
  collection.find({
    fob_ID: newList[0],
    hub_ID: newList[1],
  }).toArray(function(err, results) {
    console.log("RES");
    console.log(results);
    // console.log(results[0].Person);
    collection.insert({
      "fob_ID": newList[0],
      "hub_ID": newList[1],
      "Person": results[0].Person,
      "Time": "9999",
      "Location": results[0].Location,
      "Presence": "1"
    })
  });

}

var updatePresence = function() {
  console.log("PRESENCE: ")
  collection.find({
    Presence: "1"
  }).toArray(function(err, res) {
    console.log("DO YOU RUN");
    if (err) {
      console.error(err);
    } else {
      console.log(res);
      //  response.send("192.unique.host/bleh", results)
    }
  });
}

//
// collection.find({
//   Presence: "1"
// }).toArray(function(err, res) {
//   console.log("DO YOU RUN");
//   if (err) {
//     console.error(err);
//   } else {
//     console.log(res);
//     //  response.send("192.unique.host/bleh", results)
//   }
// });

server.bind(PORT, HOST);

http.listen(port, function() {
  console.log('listening on *:' + port);
});