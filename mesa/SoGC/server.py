from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer

from .portrayal import portrayCell
from .model import SoGC


# Make a world that is 50x50, on a 250x250 display.
canvas_element = CanvasGrid(portrayCell, 30, 30, 1500, 1500)

#inputs
paths = [input("Enter Jason Path: \n"), input("Enter Sol Path: \n")]

tracked = []
t = input("Track id: ")
while t != '':
    tracked.append(int(t))
    t = input("Track id: ")

server = ModularServer(
    SoGC, [canvas_element], "S10ppy J035", {"height": 50, "width": 50, "paths": paths , "tracked": tracked}
)
