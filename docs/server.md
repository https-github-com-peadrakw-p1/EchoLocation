# Server

This document descibres how to setup and connect to the linux server created on [Digital Ocean](https://www.digitalocean.com/).

## Connecting to the server

The server is currently running on digital ocean. The servers details are as follows:
```
ssh root@159.65.234.255
password: AMR_Group4
```

** NOTE: This server was shut down December, 11, 2018 **

## Running Web Server

Running the web server can be done using the command:

```
node server.js
```

You can then run the python script on your computer to access the web server:

```
python server_conncetion_test.y'
```

** This has only been tested on python2 **

## Dependencies

Once the droplet has been created I needed to install the following in order to allow the server to work.

```
sudo apt-get install node-js
```


## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt-carl](https://github.com/hildebrandt-carl)