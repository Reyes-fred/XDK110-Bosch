# boschxdkmqttiotamam

An example of fetching JSON data over MQTT and sending with MAM to the Tangle
This repo is a fork of Christopher Aldave's repo: https://github.com/chrislaive/MAMExample
> This example is made in JavaScript, be sure that you have NodeJS and NPM for used this repository
And is tested with my Bosch XDK JSON data

## Installation

Clone this repository and then execute
```
npm install
```

## Getting Started

After you've successfully installed you can execute index for a simple user experience

```
node index.js [sec] [mode]
```

sec: time for loop in seconds

mode: can be 3 (Public: 1, Private: 2, Restricted: 3)

i.e. (Send data in private mode each 15 seconds)
```
    node index.js 15 2
```

To use with Rck SensorNode please use 

```
node indexSensorNode.js [seg] [MAMMode]
```


## Fetching Data

If you want to fetch your data that is on tangle you can run respective file for fetch inside the fetchData folder

```
node fetch.js [root]
```

root: root since you want to fetch

i.e. (for our last example sending data in private mode each 15 seconds)

    node fetchPrivate.js QUYVAZAGSLKPBSXFUWGHGBSTJUDR9KXHPFGF9WXMYFHYULSHEFYXAWDYRHKNSA9GVRFWKALVCUSETQDDJ


> Also you can go to [MAM-explorer](https://iota-mam-explorer.now.sh/#/) and start fetching the data

