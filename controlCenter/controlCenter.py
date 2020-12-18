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
from cgshop2021_pyutils import Instance
import json
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

    def run_all(self, print_only_success=False, stop_on_success=False):
        # Run init algos
        for i in self.init_algos:
            res = i.run()
            if stop_on_success and res.out["result"] == SolutionResult.SUCCESS.name:
                self.solutions.append(res)
                break
            self.solutions.append(res)

        self.print_solutions(print_only_success)
        return self.analyze()

    def run_an_init_algo(self, algo_id: int) -> Solution:
        # TODO: After initAlgo will be ready
        return self.init_algos[algo_id].run()

    def run_an_optimization_algo(self, solution: Solution, opt_algo: OptimizationAlgo):
        # TODO
        pass

    def __init_init_algos(self):
        '''
        self.init_algos.append(LeftPillar(self.name,
                                          self.grid,
                                          self.targets,
                                          self.max_makespan,
                                          self.max_sum,
                                          self.preprocess))
        '''

        for i in range(12):
            self.init_algos.append(BFS(self.name, self.grid, self.targets, self.max_makespan, self.max_sum, self.preprocess, "_"+str(i)))

    def print_solutions(self, print_only_success):
        try:
            os.mkdir(self.solution_path)
        except:
            pass

        for i in range(len(self.solutions)):
            if print_only_success and self.solutions[i].out["result"] != SolutionResult.SUCCESS.name:
                continue
            out_file_name = self.solution_path + self.name + "_" + self.init_algos[i].name + "_" + self.solutions[i].out["result"] + ".json"
            self.solutions[i].output(out_file_name)

    def analyze(self):
        NUM_OF_DIFFERENT_ALGO = 2
        DIFFERENT_ALGO_BORDER_INDECIES = [1]
        hist = [False] * NUM_OF_DIFFERENT_ALGO
        hist_index = 0
        passed = False
        for i in range(len(self.solutions)):
            if i in DIFFERENT_ALGO_BORDER_INDECIES:
                if passed:
                    hist[hist_index] = True
                hist_index += 1
                passed = False

            passed = passed or self.solutions[i].out["result"] == SolutionResult.SUCCESS.name

        if passed:
            hist[hist_index] = True

        return hist
