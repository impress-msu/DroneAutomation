import threading
#from land_handling_final import *
from LandingServer import *
from rpi_landing_test import *

# [Corrected X, Corrected Y, Corrected Z]

def transferMain(port):
    global finalCoords
    global correctionData

    landingThread = threading.Thread(target = land_handling)
    landingThread.start()

    serverThread = threading.Thread(target = dataServer, args=(port,))
    serverThread.start()

    while(getLandedFlag() == False):
        time.sleep(0.001)
        setData(getCoords())
    
    setData(getCoords())
    landingThread.join()
    serverThread.join()
    return

transferMain(5563)