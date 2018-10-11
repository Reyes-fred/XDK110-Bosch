# Original Repo

This repo is a fork of Christopher Aldave's repo: https://github.com/chrislaive/MAMExample

## Installation

Give the next command
```
npm install
```

## Getting Started

To start application you can execute.

```
node index.js [sec] [mode]
```

sec: time for loop in seconds

mode: can be 3 (Public: 1, Private: 2, Restricted: 3)

Ex. (Send data in private mode each 15 seconds)
```
    node index.js 15 2
```

## Fetching Data

To fetch your data that is on tangle you can run respective file for fetch inside the fetchData folder

```
node fetch.js [root]
```

root: root since you want to fetch

i.e. (for our last example sending data in private mode each 15 seconds)

    node fetchPrivate.js QUYVAZAGSLKPBSXFUWGHGBSTJUDR9KXHPFGF9WXMYFHYULSHEFYXAWDYRHKNSA9GVRFWKALVCUSETQDDJ

## TIP

You can go to [MAM-explorer](https://iota-mam-explorer.now.sh/#/) and start fetching the data

