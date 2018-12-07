import socket
import time

ipAddress = '159.65.234.255'
# Used for testing
#ipAddress = '127.0.0.1'

portNumber = 4242

print("Starting a connection to server")
print("Attempting to connect to: %s:%d" % (ipAddress, portNumber))

sock = socket.socket()

try:
    sock.connect((ipAddress, portNumber))
except:
    print("Connection failure")
    exit(0)

print("Connection success")

print("Sending message to server")
sock.send("This is a test message")

print("Waiting for a responce")
return_string = sock.recv(256)

print("Received responce")
print(return_string)

print("Closing connection")
sock.close()