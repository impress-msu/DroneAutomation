from pad_plot import Pad_Plot
import time
pad = Pad_Plot(16)
#pad.plot_drone(10, 50, 100)

data = ['n','n', 'n']
landedFlag = False
vehicle = 0
def setData(newData):
    global data
    data = newData
    return

def setLandedFlag(flag):
    global landedFlag
    landedFlag = flag
    return

def plot():
    while True:
        try:
            coords = [int(data[0]), int(data[1]), int(data[2])] # in cm
            # do the plot
            pad.plot_drone(coords[0], coords[1], coords[2])

        except Exception as e:
            print(e)
            #print(data)
            pass




