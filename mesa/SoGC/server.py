from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer

from .portrayal import portrayCell
from .model import SoGC
import json



#inputs
paths = [input("Enter Jason Path: \n"), input("Enter Sol Path: \n")]

file = open(paths[0], "r")
name = str(json.load(file)["name"])
first_size_index = name.rfind('x') + 1
last_size_index = first_size_index + (name[first_size_index:]).find('_')

size = int(name[first_size_index:last_size_index])
display = 50*size

# Make a world that is 50x50, on a 250x250 display.
canvas_element = CanvasGrid(portrayCell, size, size, display, display)

tracked = []
t = input("Track id: ")
while t != '':
    tracked.append(int(t))
    t = input("Track id: ")

server = ModularServer(
    SoGC, [canvas_element], "S10ppy J035", {"height": size, "width": size, "paths": paths , "tracked": tracked}
)
