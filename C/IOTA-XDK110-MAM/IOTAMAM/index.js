'use strict'

const MAM_public = require('./lib/attachDataPublic.js')
const MAM_private = require('./lib/attachDataPrivate.js')
const MAM_restricted = require('./lib/attachDataRestricted.js')
const IOTA = require('iota.lib.js')
const iota = new IOTA()
const mqtt = require ('mqtt');

//var client  = mqtt.connect('mqtt://1234:1234@broker.hivemq.com:1883');
var client  = mqtt.connect('mqtt://xdk110:xdk110@broker.hivemq.com:1883');
var jsonData = null;

function getmyjson(myjson){
  jsonData = JSON.parse(myjson);
//  console.log(jsonData);
}

//connect and subscribe to topic
client.on('connect', function () {
  client.subscribe('XDK110/sensor/status');

//  console.log('client has subscribed successfully');
});


// get data
client.on('message', function (topic, message){
  //console.log('Received message %s %s', topic, message)
  getmyjson(message);
});

let timeLoop,date,
	i=1

if( process.argv[2] == undefined){          //Getting the time in seconds for the loop
  timeLoop = 60000                       //default 1 minute
} else {
  timeLoop = process.argv[2]*1000
}


//Create a JSON as message

function start(){
	const time = Date.now();
        let message = { 'Message' : i,
			'id' : jsonData.Ident,
			'location' : {'lat' : 20.674605, 'lng' : -103.393228},
			'timestamp' : time,
			'humidity' :	jsonData.Humidity,
			'pressure' :    jsonData.Pressure,
			'temperature' : jsonData.Temperature,
			'light' : jsonData.Light,
			'noise' : jsonData.Noise*,};
	switch(process.argv[3]){								//Getting the mode of the stream (Public:1, Private:2, Restricted: 3)
		case '1': MAM_public.attach(message);break;
		case '2': MAM_private.attach(message);break;
		case '3': MAM_restricted.attach(message);break;
		default: MAM_public.attach(message)
	}
	console.log('Start sending data to Tangle...')
	let messageS = JSON.stringify(message)
	console.log('Message: %s',messageS)
	console.log('Message in trytes: ' + iota.utils.toTrytes(messageS))
	console.log('--------------------------------------------------------------------------------------------------------------------')
	i++
}

setInterval(function(){
	start()
},timeLoop)


