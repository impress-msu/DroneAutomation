import matplotlib.pyplot as plt
import numpy as np

class Pad_Plot:

    def __init__(self, drone_radius, clearance = 5, pad_radius = 76.2,
                 px_cm = 39, px_size = 6000):
        self.drone_radius = drone_radius
        self.clearance = clearance
        self.pad_radius = pad_radius
        self.px_cm = px_cm
        self.px_size = px_size

    def axis_transform(self, x_cm, y_cm):
        x_px = x_cm * self.px_cm
        y_px = y_cm * self.px_cm
    
        x_coord = (self.px_size/2) + x_px
        y_coord = (self.px_size/2) - y_px

        return x_coord, y_coord

    def heading_transform(self, x_coord, y_coord, heading):
        heading_rad = np.deg2rad(90 - heading) 
        rise = self.drone_radius*self.px_cm*np.sin(heading_rad)
        run = self.drone_radius*self.px_cm*np.cos(heading_rad)
        return run, rise

    def is_safe(self, x_cm, y_cm ):
        dist = np.sqrt( x_cm**2 + y_cm**2 ) + 16
        if dist <= ( self.pad_radius - self.clearance ):
            return 'g'
        else:
            return 'r'

    def plot_drone( self, x_cm, y_cm, heading):
        img = plt.imread("landing_pad.png")
        fig, ax = plt.subplots()
        ax.imshow(img)

        x, y = self.axis_transform( x_cm, y_cm)
        clr = self.is_safe(x_cm, y_cm)
        drone_span = plt.Circle((x, y), self.drone_radius*self.px_cm, color=clr, fill=False)
        ax.add_artist(drone_span)
        x_slope, y_slope = self.heading_transform(x, y, heading)
        plt.arrow(x, y, x_slope, -y_slope, length_includes_head=True,
                  head_width=50, head_length=100, color='b')
        plt.show()

