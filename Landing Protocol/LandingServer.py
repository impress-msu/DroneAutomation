###
# Austin Flynt
#
# RUN IN PYTHON 2
#
###

import socket
import random
import time

correctionData = []

def setData(Cdata):
    global correctionData
    correctionData = Cdata
    return

# This should be launched in its own thread. It will need to invoke an parameter update on line 28
def dataServer(port):  

    global correctionData

    # Open a TCP socket to establish and verify a connection with the ground control station
    keySock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Variable for the UDP socket port. Will be given to the client
    recv_port = ""

    # The key that indicates that the client is valid
    desiredKey = "117\n"

    # Establish TCP connection with the client
    keySock.bind(('',port))
    keySock.listen(1)
    conn,addr = keySock.accept()

    # Get client address
    address = conn.getsockname()[0]

    # Wait for the verification key
    while(conn.recv(256) != desiredKey):
        print "Invalid key"
        continue

    # Start the UDP data server here
    dataSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dataSock.bind(('',0))
    port_r = dataSock.getsockname()[1]

    # Send the new port to the client
    conn.send(str(port_r))

    # Notify that the connection has been established
    print "Handshake received. UDP on port" , port_r

    # Close TCP socket
    conn.close()

    # Notifies of an end of transfer
    eot = False;

    # While the server hasn't reached an end of transfer flag
    while eot == False:

        time.sleep(0.01)
 
        # Get current data
        data = correctionData

        # For each element in the data array
        for i in range(len(data)):

            # Wait for ready command
            dummy,addr = dataSock.recvfrom(1024)

            # If data = end, raise the end of transfer flag
            if(data[i] == "end"):
                dataSock.sendto(data[i], addr)
                eot = True
                break;

            # Insert index indicator
            message = "<"+str(i)+">"+str(data[i])

            # Send to the ground control client
            dataSock.sendto(message, addr)

    # Close the UDP socket
    dataSock.close()
    return
