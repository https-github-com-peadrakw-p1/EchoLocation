var net = require('net');
var PORT = 4242;

var TotalMessages = 0;
// Create a server instance, and chain the listen function to it
// The function passed to net.createServer() becomes the event handler for the 'connection' event
// The sock object the callback function receives UNIQUE for each connection
net.createServer(function(socket) {
    
    // Add a 'data' event handler to this instance of socket
    socket.on('data', function(data) {
        
        console.log("Recieved communication from " + socket.remoteAddress + "--" + data) ;
        
        // Keep track of total messages
        TotalMessages = TotalMessages + 1 ;
        console.log("Message number: " + TotalMessages) ;
        console.log(data.toString()) ;
        socket.write("GOT MESSAGE") ;
    });
    
    // Add a 'close' event handler to this instance of socket
    socket.on('close', function(data) {
        console.log("Closed socket") ;
    });

    // If there is an error event
    socket.on("error", function(err) {
        console.log("There was an error") ;
        console.log(err.stack) ;
    });

    // If there is an error event
    socket.on("end", function(data) {
        console.log("There was an end") ;
    });
    
}).listen(PORT)


console.log('Server listening on port: ' + PORT);