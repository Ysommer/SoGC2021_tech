from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer

from .portrayal import portrayCell
from .model import SoGC


# Make a world that is 50x50, on a 250x250 display.
canvas_element = CanvasGrid(portrayCell, 20, 20, 500, 500)

#inputs
paths = [input("Enter Jason Path: \n"), input("Enter Sol Path: \n")]

tracked = []
t = int(input("Track id: "))
while t != -1:
    tracked.append(t)
    t = int(input("Track id: "))

server = ModularServer(
    SoGC, [canvas_element], "S10ppy J035", {"height": 50, "width": 50, "paths": paths , "tracked": tracked}
)
