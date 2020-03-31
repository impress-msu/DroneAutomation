import threading
# from land_handling_final import *
from rpi_server import *
from landed_protocol import *


# [Corrected X, Corrected Y, Corrected Z]

def transferMain(port):
    global finalCoords
    global coords

    landingThread = threading.Thread(target=landed_protocol)
    landingThread.setDaemon(True)
    landingThread.start()

    serverThread = threading.Thread(target=dataServer, args=(port,))
    serverThread.setDaemon(True)
    serverThread.start()

    while (getLandedFlag() == False):
        time.sleep(0.001)
        setData(getCoords())


    setData(getCoords())
    #landingThread.join()
    #serverThread.join()
    return


transferMain(5563)