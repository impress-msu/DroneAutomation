###
# Austin Flynt
#
# RUN IN PYTHON 2
#
###

import socket
import time

recvData = ['n','n','n']
readyFlag = False
landedFlag = False
startCorrections = False

def getCorrectionsFlag():
    global startCorrections
    return startCorrections

def getLandedFlag():
    global landedFlag
    return landedFlag

def setReadyFlag():
    global readyFlag
    readyFlag = True

def getData():
    global recvData
    return recvData


# This should be ran on the ground control station. Add data to a queue for access from other threads
def dataClient(address, port):
    
    global recvData
    global readyFlag
    global landedFlag
    global startCorrections

    # Open TCP socket to verify that this client is valid
    keySock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # This will be the port for the UDP data transfer. Given by the server
    recv_port = ""

    # This is the key that proves this client is valid
    desiredKey = "117\n"

    # Establish a TCP connection with the server
    keySock.connect((address, port))

    # Send the key to the server
    keySock.sendto(desiredKey,(address,port))

    # Receive the port for UDP data transfer
    recv_port = keySock.recv(256)

    # Displays the port for UDP transfer
    print(recv_port)

    # Close the TCP socket
    keySock.close()

    # Conver the received port to an integer
    recv_port = int(recv_port)

    # Open a UDP socket for data transfer
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Tuple for UDP server information
    server_address = (address, recv_port)

    # Establish connection to the UDP server
    sock.connect(server_address)

    # Start corrections stream
    startCorrections = True

    while True:
        try:

            # Send ready command
            #while(readyFlag == False):
             #   print "test"
              #  continue

            sock.sendto("ready",server_address)

            readyFlag = False

            # Receive data from the server
            val = sock.recv(1024)

            # If the end of transfer value is sent raise an exception to exit the receiving loop
            if(val == "end"):
                landedFlag = True
                raise Exception

            # Display the sent data
            recvData[int(val[1])] = val[3:]

        except Exception as e:

            # Print the exception
            print(e)

            # Close the UDP socket
            sock.close()

            # Exit the loop
            break

    while(readyFlag == False):
        continue

    return




