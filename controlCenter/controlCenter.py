from infrastructure.grid import Grid
from infrastructure.cell import Cell
from infrastructure.robot import Robot
from infrastructure.solGrid import SolGrid
from dataCollection.preprocess import *
from dataCollection.postprocess import *
from algos.initAlgo import *
from algos.init_algos.BFS import *
from algos.optimizationAlgo import *
from algos.OptimizationShell import *
from solution.solution import *
from cgshop2021_pyutils import Instance
import json
import os
from math import ceil
import traceback
from typing import List


class ControlCenter:
    def __init__(self, instance: Instance, solution_path=None, max_makespan=-1, max_sum=-1):
        # inputs
        self.solution_path = solution_path
        self.instance = instance
        self.name = instance.name

        try:
            first_size_index = self.name.rfind('x') + 1
            last_size_index = first_size_index + (self.name[first_size_index:]).find('_')
            self.size = int(self.name[first_size_index:last_size_index])
        except:
            self.__set_size()

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
        self.optimization_shells = []
        self.solutions = []
        self.max_makespan = max_makespan
        self.max_sum = max_sum

        try:
            os.mkdir(self.solution_path)
        except:
            pass

    def run_all(self, print_only_success=False, stop_on_success=False, validate=False):
        self.run_all_init_algos(print_only_success, stop_on_success, validate)
        self.run_all_opt_algos(print_only_success, stop_on_success, validate)

        return  # self.analyze()

    def run_all_init_algos(self, print_only_success=False, stop_on_success=False, validate=False):
        # Run init algos
        for i in self.init_algos:
            print("Algo:", i.name, "starts running")
            try:
                res = i.run()
                print(res)
            except Exception as e:
                print("Failure in :", i.name, "| error: ", e)
                traceback.print_exc()
                continue
            try:
                if validate and res.out["result"] == SolutionResult.SUCCESS.name:
                    self.validator(res)
            except Exception as e:
                print("Failure in :", i.name, "| error: ", e)
            print("Algo:", i.name, "done with solutions", res.out["result"])
            self.solutions.append(res)
            if (not print_only_success) or res.out["result"] == SolutionResult.SUCCESS.name:
                self.print_last_solution()

            if stop_on_success and res.out["result"] == SolutionResult.SUCCESS.name:
                break

            print("\n")

    def run_all_opt_algos(self, print_only_success=False, stop_on_success=False, validate=False):
        new_solutions = []
        for sol in self.solutions:
            if sol.out["result"] != SolutionResult.SUCCESS.name:
                continue
            for shell in self.optimization_shells:
                opt_algo = shell.algo_class(
                    self.instance.name,
                    sol,
                    self.robots,
                    self.targets,
                    self.instance.obstacles,
                    self.preprocess,
                    shell.algo_name,
                    shell.print_info,
                    shell.data_bundle
                )
                print("Algo:", opt_algo.name, "starts running")

                try:
                    res = opt_algo.run()
                    print(res)
                except Exception as e:
                    print("Failure in :", opt_algo.name, "| error: ", e)
                    traceback.print_exc()
                    continue
                try:
                    if validate and res.out["result"] == SolutionResult.SUCCESS.name:
                        self.validator(res)
                except Exception as e:
                    print("Failure in :", opt_algo.name, "| error: ", e)
                print("Algo:", opt_algo.name, "done with solutions", res.out["result"])
                new_solutions.append(res)
                if (not print_only_success) or res.out["result"] == SolutionResult.SUCCESS.name:
                    self.print_last_solution(new_solutions)

                if stop_on_success and res.out["result"] == SolutionResult.SUCCESS.name:
                    break

                print("\n")

        self.solutions += new_solutions

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

    def add_opt_algo(self, algo: classmethod, name="_", print_info=True, data_bundle=None):
        self.optimization_shells.append(OptimizationShell(algo, name, print_info, data_bundle))

    def add_solution(self, solution: Solution, to_validate):
        self.solutions.append(solution)

    def print_last_solution(self, list= None):
        if list:
            list[-1].output(self.solution_path, self.name)
        else:
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

    def __set_size(self):
        max_x = 0
        max_y = 0
        for i in self.instance.start:
            max_x = max(max_x, i[0])
            max_y = max(max_y, i[1])
        for i in self.instance.target:
            max_x = max(max_x, i[0])
            max_y = max(max_y, i[1])
        for i in self.instance.obstacles:
            max_x = max(max_x, i[0])
            max_y = max(max_y, i[1])

        # Todo: check how bad it will be to support unsymmetrical grid
        self.size = (ceil(max(max_x, max_y)/10)) * 10

    def validator(self, solution: Solution):
        sol_grid = SolGrid(self.robots, self.instance.obstacles, solution, validate=True)
        assert sol_grid.validate_solution(self.targets), "The robots still haven't found what they're looking for stupid ass"