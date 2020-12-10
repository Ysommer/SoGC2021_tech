from .infrastructure.grid import *
from .infrastructure.cell import *
from .infrastructure.robot import *
from  .dataCollection.preprocess import *
from  .dataCollection.postprocess import *
from .algos.initAlgo import *
from .algos.initAlgo import *
from .algos.optimizationAlgo import *
from .solution.solution import *
import json


class ControlCenter():
    def __init__(self, paths=None, max_make_span=-1, max_sum=-1):
        # inputs
        if paths is None:
            paths = [input("Enter Jason Path: \n"), input("Enter Solution Path: \n")]

        self.paths = paths
        self.inputDict = json.load(open(paths[0], "r"))
        self.name = str(self.inputDict["name"])

        first_size_index = self.name.rfind('x') + 1
        last_size_index = first_size_index + (self.name[first_size_index:]).find('_')
        self.size = int(self.name[first_size_index:last_size_index])

        self.preprocess = Preprocess(self.inputDict)
        self.postprocess = Postprocess(self.inputDict)

        self.robots = []
        for i in range(len(self.inputDict["starts"])):
            start = self.inputDict["starts"][i]
            end = self.inputDict["targets"][i]
            pos = (start[0], start[1])
            target_pos = (end[0], end[1])
            self.robots.append(Robot(i, pos, target_pos))

        self.grid = Grid(self.size, self.robots, self.inputDict["obstacles"])

        self.init_algos = List[InitAlgo]
        self.optimization_algos = List[OptimizationAlgo]
        self.solutions = List[Solution]
        self.max_make_span = max_make_span
        self.max_sum = max_sum

    def run_all(self):
        # TODO
        pass

    def run_an_init_algo(self, algo_id: int) -> Solution:
        # TODO: After initAlgo will be ready
        return self.init_algos[algo_id].run()

    def run_an_optimization_algo(self, solution: Solution, opt_algo: OptimizationAlgo):
        # TODO
        pass
