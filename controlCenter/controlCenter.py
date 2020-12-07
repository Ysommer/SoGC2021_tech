from .infrastructure.grid import *
from .infrastructure.cell import *
from .infrastructure.robot import *
from  .dataCollection.preprocess import *
from  .dataCollection.postprocess import *
from .algos.algo import *
from .algos.initAlgo import *
from .algos.optimizationAlgo import *
from .solution.solution import *
import json


class ControlCenter():
    def __init__(self, paths=None, boundaries_percentage=0, max_make_span = -1, max_sum = -1):

        # inputs
        if paths is None:
            paths = [input("Enter Jason Path: \n"), input("Enter Solution Path: \n")]

        self.paths = paths
        self.inputDict = json.load(open(paths[0], "r"))
        self.name = str(self.inputDict["name"])

        first_size_index = self.name.rfind('x') + 1
        last_size_index = first_size_index + (self.name[first_size_index:]).find('_')
        self.size = int(self.name[first_size_index:last_size_index])

        self.boundaries_size = int(self.size * boundaries_percentage)
        self.real_size = self.size + self.boundaries_size * 2

        self.preprocess = Preprocess(self.inputDict)
        self.postprocess = Postprocess(self.inputDict)

        self.grid = Grid(self.real_size)
        self.initAlgorithms = []
        self.optimizationAlgorithms = []
        self.solutions = []
        self.max_make_span = max_make_span
        self.max_sum = max_sum

    def runAll(self):
        #TODO
        return

    def runAnInitiateAlgorithm(self, initiateAlgorithm:InitiateAlgorithm):
        #TODO
        return

    def runAnOptimizationAlgorithm(self, solution:Solution ,optAlgorithm:OptimizationAlgorithm):
        #TODO
        return