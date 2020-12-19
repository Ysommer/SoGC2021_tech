from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from .defines import *
from .portrayal import portrayCell
from .model import SoGC
import json


""" ### CHANGE PARAMETERS FROM define.py AND NOT FROM HERE ### """

#inputs
paths = [
    "../instances/instances_01/uniform/"+Instance_name+".instance.json",
    "../solutions/"+Instance_name+"/"+Sol_name
]

file = open(paths[0], "r")
name = str(json.load(file)["name"])
first_size_index = name.rfind('x') + 1
last_size_index = first_size_index + (name[first_size_index:]).find('_')

original_size = int(name[first_size_index:last_size_index])
full_size = original_size * BOUNDARIES_FACTOR
display = 50*full_size

# Make a world that is 50x50, on a 250x250 display.
canvas_element = CanvasGrid(portrayCell, full_size, full_size, display, display)

tracked = []
t = input("Track id: ")
while t != '':
    tracked.append(int(t))
    t = input("Track id: ")

server = ModularServer(
    SoGC, [canvas_element], "S10ppy J035", {"height": full_size, "width": full_size, "paths": paths , "tracked": tracked}
)
