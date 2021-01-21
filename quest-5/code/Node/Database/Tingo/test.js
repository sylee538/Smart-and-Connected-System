var Db = require('tingodb')().Db;
assert = require('assert');
fs = require('fs');

var db = new Db('./', {});
// Fetch a collection to insert document into
var collection = db.collection("newData.json");
// Insert a single document
fs.readFile('data2.txt', (err, data) => {
    if (err) throw err;
    var stringData = data.toString();
    var list = stringData.split("\n");
    list.shift()
    list.pop(-1)
    for (var i = 0; i < list.length; i++) {

        list[i] = list[i].split("\t");
        list[i] = {
            "Time": list[i][0],
            "fob_ID": list[i][1],
            "hub_ID": list[i][2],
            "Name": list[i][3],
	    "Checked-In": list[i][4]
        }
        collection.insert(list[i])
    }
    collection.find({
        hub_ID: "1"
    }).toArray(function(err, results) {
        if (err) {
            console.error(err);
        } else {
            console.log(results);
	    //  response.send("192.unique.host/bleh", results)
        }
    });
});

//app.get("/receiver", function(req, res){

//	req.body.what_i_received
//})


