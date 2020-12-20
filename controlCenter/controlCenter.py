from infrastructure.grid import Grid
from infrastructure.cell import Cell
from infrastructure.robot import Robot
from dataCollection.preprocess import *
from dataCollection.postprocess import *
from algos.initAlgo import *
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

        try:
            os.mkdir(self.solution_path)
        except:
            pass

    def run_all(self, print_only_success=False, stop_on_success=False):
        # Run init algos
        for i in self.init_algos:
            print("Algo:", i.name,"starts running")
            try:
                res = i.run()
            except:
                print("Failure in :", i.name)
                continue
            print("Algo:", i.name, "done with solutions", res.out["result"])
            self.solutions.append(res)
            if (not print_only_success) or res.out["result"] == SolutionResult.SUCCESS.name:
                self.print_last_solution()

            if stop_on_success and res.out["result"] == SolutionResult.SUCCESS.name:
                break

            print("\n")

        return self.analyze()

    def run_an_init_algo(self, algo_id: int) -> Solution:
        # TODO: After initAlgo will be ready
        return self.init_algos[algo_id].run()

    def run_an_optimization_algo(self, solution: Solution, opt_algo: OptimizationAlgo):
        # TODO
        pass

    def add_init_algo(self, algo: classmethod, name="_", print_info=True, data_bundle=None):
        self.init_algos.append(
            algo(
                self.name,
                self.grid,
                self.targets,
                self.max_makespan,
                self.max_sum,
                self.preprocess,
                name=name,
                print_info=print_info,
                data_bundle=data_bundle))

    def __init_init_algos(self):
        pass
        """
        self.init_algos.append(LeftPillar(self.name,
                                          self.grid,
                                          self.targets,
                                          self.max_makespan,
                                          self.max_sum,
                                          self.preprocess))

        for i in range(10):
            self.init_algos.append(BFS(self.name, self.grid, self.targets, self.max_makespan // 2, self.max_sum // 2, self.preprocess, "_"+str(i)))

        self.init_algos.append(OutAndInBFS(self.name,
                                          self.grid,
                                          self.targets,
                                          self.max_makespan,
                                          self.max_sum,
                                          self.preprocess,
                                          name= "_default"))
        
        self.init_algos.append(OutAndInBFS(self.name,
                                            self.grid,
                                            self.targets,
                                            self.max_makespan,
                                            self.max_sum,
                                            self.preprocess,
                                            start_fill_from=(self.size//2, self.size//2),
                                            reverse_fill=False,
                                           name="_travel_from_center"))
                                           """

    def print_last_solution(self):
        self.solutions[-1].output(self.solution_path, self.name)

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
