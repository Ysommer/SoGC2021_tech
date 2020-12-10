from infrastructure.grid import Grid
from infrastructure.cell import Cell
from infrastructure.robot import Robot
from dataCollection.preprocess import *
from dataCollection.postprocess import *
from algos.initAlgo import *
from algos.init_algos.leftPillar import *
from algos.init_algos.BFS import *
from algos.optimizationAlgo import *
from solution.solution import *
import json


class ControlCenter():
    def __init__(self, paths=None, max_makespan=-1, max_sum=-1):
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
        self.targets = []
        for i in range(len(self.inputDict["starts"])):
            start = self.inputDict["starts"][i]
            end = self.inputDict["targets"][i]
            pos = (start[0], start[1])
            target_pos = (end[0], end[1])
            self.robots.append(Robot(i, pos, target_pos))
            self.targets.append(self.inputDict["targets"][i])

        self.grid = Grid(self.size, self.robots, self.inputDict["obstacles"])

        self.init_algos = []
        self.optimization_algos = []
        self.solutions = []
        self.max_makespan = max_makespan
        self.max_sum = max_sum

        self.__init_init_algos()

    def run_all(self):
        # Run init algos
        for i in self.init_algos:
            self.solutions.append(i.run())

        self.printSolutions()

    def run_an_init_algo(self, algo_id: int) -> Solution:
        # TODO: After initAlgo will be ready
        return self.init_algos[algo_id].run()

    def run_an_optimization_algo(self, solution: Solution, opt_algo: OptimizationAlgo):
        # TODO
        pass

    def __init_init_algos(self):
        self.init_algos.append(BFS(self.name, self.grid, self.robots, self.targets, self.max_makespan, self.max_sum, self.preprocess))

    def printSolutions(self):
        for i in range(len(self.solutions)):
            out_file_name = self.paths[1] + self.name + str(i) + ".json"
            self.solutions[i].print(out_file_name)
