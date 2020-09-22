###
# Austin Flynt
#
# RUN IN PYTHON 2
#
### 

import threading
from LandingClient import *
from drone_kit_landing_velocity_vector import *
#from GroundControlSim import *

# [Corrected X, Corrected Y, Corrected Z]

def transferMain(address, port):

    global finalCoords
    global correctionData

    f = open("LandingData1.log", "w")

    landingThread = threading.Thread(target = runVehicle)

    serverThread = threading.Thread(target = dataClient, args=(address, port,))
    serverThread.start()

    while(getCorrectionsFlag() == False):
        continue
    
    landingThread.start()

    while(getLandedFlag() == False):
        setReadyFlag()
        setData(getData())
        dataStr = getData()
        dataStr = ("%d %d %d\n", dataStr[0], dataStr[1], dataStr[2])
        f.write(str(dataStr))
    
    setReadyFlag()
    setLandedFlag(True)
    f.close()
    landingThread.join()
    serverThread.join()
    
    return

transferMain("192.168.43.195",5563)
