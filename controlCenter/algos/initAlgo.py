import abc
from solution.solution import Solution
from infrastructure.grid import Grid
from defines import *
from infrastructure.robot import Robot
from copy import deepcopy
from utils import Timer

class InitAlgo(abc.ABC):

    def __init__(self, instance_name: str, grid: Grid, targets: list, max_makespan: int = None, max_sum: int = None, preprocess=None, name="", print_info=False):
        self.name = name
        self.instance_name = instance_name
        self.grid = deepcopy(grid)
        self.robots = self.grid.robots
        self.targets = targets
        self.max_makespan = max_makespan
        self.max_sum = max_sum
        self.preprocess = preprocess
        self.current_turn = 0
        self.current_sum = 0
        self.solution = Solution(instance_name, self.name)
        self.print_info = print_info
        self.run_timer = Timer(self.name + " runtime")

        self.arrived_order = []
        self.time_arrived = [-1] * len(self.robots)

        self.force_stop = False

    @abc.abstractmethod
    def step(self) -> int:
        """
        defines a simultaneous step in the algorithm and updates solution.
        :return number of moved robots
        """
        ...

    def run(self) -> Solution:
        """
        defines the execution of the algorithm.
        should look something like this:
        while(True):
            if self.max_makespan is not None and self.max_makespan <= self.current_turn:
                solution.result = SolutionResult.EXCEEDED_MAX_MAKESPAN
                break
            if self.max_sum is not None and self.max_sum <= self.current_sum:
                solution.result = SolutionResult.EXCEEDED_MAX_SUM
            self.step()
            if grid.solution_found:
                solution.result = SUCCESS
                break
            self.current_turn += 1

        return solution
        """
        self.run_timer.start()
        last_milestone = 0
        while True and not self.force_stop:
            if self.max_sum != -1 and self.current_sum > self.max_sum:
                self.solution.put_result(SolutionResult.EXCEEDED_MAX_SUM, self.current_turn, self.current_sum)
                break

            if self.max_makespan != -1 and self.current_turn > self.max_makespan:
                self.solution.put_result(SolutionResult.EXCEEDED_MAX_MAKESPAN, self.current_turn, self.current_sum)
                break

            if self.grid.solution_found():
                self.solution.put_result(SolutionResult.SUCCESS, self.current_turn, self.current_sum)
                break

            self.solution.out["steps"].append({})

            last_turn_sum = self.step()

            if last_turn_sum == 0:
                self.solution.put_result(SolutionResult.STUCK, self.current_turn, self.current_sum)
                break

            self.current_turn += 1
            self.current_sum += last_turn_sum

            if (100 * self.grid.numOfRobotsArrived) // len(self.robots) >= last_milestone + 10:
                last_milestone = (((100 * self.grid.numOfRobotsArrived) // len(self.robots)) // 10) * 10
                if self.print_info:
                    print(last_milestone,"% of robots arrived")

        if self.force_stop:
            self.solution.put_result(SolutionResult.STUCK, self.current_turn, self.current_sum)
            print("Algo run was forced to stop")
            return self.solution

        self.run_timer.end(self.print_info)
        self.solution.out["extra"]["arrival_order"] = self.arrived_order
        self.solution.out["extra"]["time_arrived"] = self.time_arrived

        return self.solution


    @staticmethod
    def move_robot_to_dir(robot_id: int, grid: Grid, direction: str, current_turn: int, solution: Solution) -> int:
        if grid.move_robot(robot_id, direction, current_turn) == EnterCellResult.SUCCESS:
            solution.update_robot(robot_id, direction, current_turn)
            return 1

        return 0
