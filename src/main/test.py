import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from util.dubinsUtil import dubins_path, int_dubins_path

from util.shapeUtil import (
    bresenham_line,
    bresenham_circle,
    bresenham_filled_circle,
    filled_square,
    square,
    bresenham_arc,
    bresenham_filled_arc,
    step_line,
    double_step_line,
)
import util.trackUtil as tu

import util.elevationUtil as eu

# from dubins import dubins_path


def plot_path(center_track, path_points,track):
    # Plotting for visual verification without lines between points
    plt.figure(figsize=(8, 8))
    x_center, y_center = zip(*center_track)
    
    x, y = zip(*path_points[0])
    
    x0, y0 = zip(*path_points[1])
    x1, y1, dx1, dy1, = zip(*track[0][0])
    x2, y2, dx2, dy2 = zip(*track[0][1])
    x3, y3, dx3, dy3 = zip(*track[1][0])
    x4, y4, dx4, dy4 = zip(*track[1][1])
    x5, y5, dx5, dy5 = zip(*track[2][0])
    x6, y6, dx6, dy6 = zip(*track[2][1])


    plt.scatter(x, y, s=10, marker="s")
    plt.scatter(x0, y0, s=10, marker="s")
    plt.scatter(x1, y1, s=10, marker="s")
    plt.scatter(x2, y2, s=10, marker="s")
    plt.scatter(x3, y3, s=10, marker="s")
    plt.scatter(x4, y4, s=10, marker="s")
    plt.scatter(x5, y5, s=10, marker="s")
    plt.scatter(x6, y6, s=10, marker="s")
    
    plt.scatter(x_center, y_center, s=10, marker="s", color="red")
    
    plt.axis("equal")
    plt.title("Dubins Path")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()


base_width = 13
track_width = 7

start = start = [(0,0,0), 0]
end = np.array([0, 70, np.pi / 3])

# start1 = np.array([0.0, 70.0, np.pi / 3])
# end1 = np.array([100.0, 70.0, np.pi / 2])

radius = 30.0

center_track = int_dubins_path(start[0], end, radius)

base, track= tu.track(center_track, start[0], end, base_width, track_width)

mask = eu.generate_elevation_mask(center_track,base_width,base[0])

# print(track_center)

    
plot_path(center_track, base, track)
