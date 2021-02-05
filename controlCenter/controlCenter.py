import functools
print = functools.partial(print, flush=True)

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
from algos.InitShell import *
from solution.solution import *
from cgshop2021_pyutils import Instance
import json
import os
from math import ceil
import traceback
from typing import List
from utils import Timer


class ControlCenter:
    def __init__(self, instance: Instance, solution_path=None, max_makespan=-1, max_sum=-1, automate_makespan_and_sum=False, print_init_sol=True):
        # inputs
        self.solution_path = solution_path
        self.instance = instance
        self.name = instance.name
        self.automate_makespan_and_sum = automate_makespan_and_sum
        self.print_init_sol = print_init_sol

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
        self.init_shells: List[InitShell] = []
        self.optimization_shells: List[OptimizationShell] = []
        self.solutions = []
        self.max_makespan = max_makespan
        self.max_sum = max_sum

        self.min_makespan = max_makespan
        self.min_sum = max_sum


        try:
            os.mkdir(self.solution_path)
        except:
            pass

    def run_all(self, print_only_success=False, stop_on_success=False, validate=False, opt_iters=1, pick_best_sum=-1):
        self.run_all_init_algos(print_only_success, stop_on_success, validate)
        if (opt_iters == 1):
            self.run_all_opt_algos(print_only_success, stop_on_success, validate)
        else:
            for i in range(opt_iters):
                if len(self.solutions) > pick_best_sum != -1:
                    self.solutions.sort(key=lambda sol: sol.out["sum"])
                    self.solutions = self.solutions[:pick_best_sum]

                prev_solutions = self.solutions.copy()
                self.run_all_opt_algos(print_only_success, stop_on_success, validate)
                self.solutions = list(set(self.solutions) - set(prev_solutions))
                if len(self.solutions) == 0:
                    break


        return  # self.analyze()

    def run_all_init_algos(self, print_only_success=False, stop_on_success=False, validate=False):
        # Run init algos
        for i in range(len(self.init_shells)):
            res = self.run_an_init_algo(i, print_only_success, validate)
            if res and stop_on_success:
                break

    def run_all_opt_algos(self, print_only_success=False, validate=False, auto_makespan=False, auto_sum=False):
        solutions_counter = len(self.solutions)

        for sol_id in range(solutions_counter):
            for shell_id in range(len(self.optimization_shells)):
                self.run_an_optimization_algo(self.solutions[sol_id], shell_id, print_only_success, validate, auto_makespan, auto_sum)

    def run_an_init_algo(self, algo_id: int, print_only_success=False, validate=False) -> bool:
        print()
        algo = self.init_shells[algo_id].algo_class(
                self.name,
                self.grid,
                self.targets,
                self.max_makespan,
                self.max_sum,
                self.preprocess,
                name=self.init_shells[algo_id].algo_name,
                print_info=self.init_shells[algo_id].print_info,
                data_bundle=self.init_shells[algo_id].data_bundle)

        print("Algo:", algo.name, "starts running")
        try:
            res = algo.run()
            print(res)
        except Exception as e:
            print("Failure in :", algo.name, "| error: ", e)
            traceback.print_exc()
            return False
        try:
            if validate and res.out["result"] == SolutionResult.SUCCESS.name:
                self.validator(res)
        except Exception as e:
            print("Failure in :", algo.name, "| error: ", e)

        print("Algo:", algo.name, "done with solutions", res.out["result"])
        if res.out["result"] != SolutionResult.SUCCESS.name:
            if not print_only_success and self.print_init_sol:
                self.print_last_solution([res])
            return False

        self.solutions.append(res)
        if self.print_init_sol:
            self.print_last_solution()


        if self.min_makespan == -1 or self.min_makespan > res.out["makespan"]:
            self.min_makespan = int(res.out["makespan"])

        if self.min_sum == -1 or self.min_sum > res.out["sum"]:
            self.min_sum = int(res.out["sum"])

        return True

    def run_an_optimization_algo(self, solution: Solution, opt_algo_id: int, print_only_success=False, validate=False, auto_makespan=False, auto_sum=False) -> bool:
        print()
        if solution.out["result"] != SolutionResult.SUCCESS.name:
            return False

        timer = Timer("opt runtime ")

        algo = self.optimization_shells[opt_algo_id].algo_class(
            self.instance.name,
            solution,
            self.robots,
            self.targets,
            self.instance.obstacles,
            self.size,
            self.preprocess,
            self.optimization_shells[opt_algo_id].algo_name,
            self.optimization_shells[opt_algo_id].print_info,
            self.optimization_shells[opt_algo_id].data_bundle)

        print("Algo:", algo.name, "starts running")

        try:
            timer.start()
            sol_res = algo.run()
            timer.end(True)
            print(sol_res)
        except Exception as e:
            print("Failure in :", algo.name, "| error: ", e)
            traceback.print_exc()
            return False
        try:
            if validate and sol_res.out["result"] == SolutionResult.SUCCESS.name:
                self.validator(sol_res)
        except Exception as e:
            print("Failure in :", algo.name, "| error: ", e)

        if sol_res.out["result"] != SolutionResult.SUCCESS.name:
            if not print_only_success:
                self.print_last_solution([sol_res])
                return False

        if solution.out["makespan"] <= sol_res.out["makespan"] and solution.out["sum"] <= sol_res.out["sum"]:
            return False

        if auto_makespan and self.min_makespan < sol_res.out["makespan"]:
            return False
        else:
            if self.min_makespan == -1 or self.min_makespan > sol_res.out["makespan"]:
                self.min_makespan = int(sol_res.out["makespan"])

        if auto_sum and self.min_sum < sol_res.out["sum"]:
            return False
        else:
            if self.min_sum == -1 or self.min_sum > sol_res.out["sum"]:
                self.min_sum = int(sol_res.out["sum"])

        print("Algo:", algo.name, "done with solutions", sol_res.out["result"])
        self.solutions.append(sol_res)
        self.print_last_solution([sol_res])
        return True

    def add_init_algo(self, algo: classmethod, name="", print_info=True, data_bundle=None):
        self.init_shells.append(InitShell(algo, name, print_info, data_bundle))

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