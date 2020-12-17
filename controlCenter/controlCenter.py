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
from cgshop2021_pyutils import InstanceDatabase, Instance
import os


class ControlCenter:
    def __init__(self, instance: Instance, solution_path=None, max_makespan=-1, max_sum=-1):
        # inputs
        self.solution_path = solution_path
        self.instance = instance
        self.name = instance.name

        first_size_index = self.name.rfind('x') + 1
        last_size_index = first_size_index + (self.name[first_size_index:]).find('_')
        self.size = int(self.name[first_size_index:last_size_index])

        self.robots = []
        self.targets = []

        for i in range(self.instance.number_of_robots):
            start = self.instance.start_of(i)
            target = self.instance.target_of(i)
            self.robots.append(Robot(i, start, target))
            self.targets.append(target)

        self.preprocess = Preprocess(self.instance, self.robots)
        self.postprocess = Postprocess(self.instance)

        self.grid = Grid(self.size, self.robots, self.instance.obstacles)

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
        self.init_algos.append(LeftPillar(self.name, self.grid, self.robots, self.targets, self.max_makespan, self.max_sum, self.preprocess))
        #self.init_algos.append(BFS(self.name, self.grid, self.robots, self.targets, self.max_makespan, self.max_sum, self.preprocess))

    def printSolutions(self):
        try:
            os.mkdir(self.solution_path)
        except:
            pass

        for i in range(len(self.solutions)):
            out_file_name = self.solution_path + self.name + "_" + self.init_algos[i].name + "_" + ".json"
            self.solutions[i].output(out_file_name)
