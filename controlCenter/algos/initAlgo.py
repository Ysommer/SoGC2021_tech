import abc
from solution.solution import Solution
from infrastructure.grid import Grid
from defines import *
from infrastructure.robot import Robot
from copy import deepcopy



class InitAlgo(abc.ABC):

    def __init__(self, instance_name: str, grid: Grid, robots: list, targets: list, max_makespan: int = None, max_sum: int = None, preprocess=None, name=""):
        self.name = name
        self.instance_name = instance_name
        self.grid = deepcopy(grid)
        self.robots = deepcopy(robots)
        self.targets = targets
        self.max_makespan = max_makespan
        self.max_sum = max_sum
        self.preprocess = preprocess
        self.current_turn = 0
        self.current_sum = 0
        self.solution = Solution(instance_name)

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
        while True:
            if self.max_sum != -1 and self.current_sum > self.max_sum:
                self.solution.put_result(SolutionResult.EXCEEDED_MAX_SUM, self.current_turn, self.current_sum)
                return self.solution

            if self.max_makespan != -1 and self.current_turn > self.max_makespan:
                self.solution.put_result(SolutionResult.EXCEEDED_MAX_MAKESPAN, self.current_turn, self.current_sum)
                return self.solution

            if self.grid.solution_found():
                self.solution.put_result(SolutionResult.SUCCESS, self.current_turn, self.current_sum)
                return self.solution

            self.solution.out["steps"].append({})

            last_turn_sum = self.step()

            if last_turn_sum == 0:
                self.solution.put_result(SolutionResult.STUCK, self.current_turn, self.current_sum)
                return self.solution

            self.current_turn += 1
            self.current_sum += last_turn_sum

    @staticmethod
    def move_robot_to_dir(robot_id: int, grid: Grid, direction: str, current_turn: int, solution: Solution) -> bool:
        if grid.move_robot(robot_id, direction, current_turn) == EnterCellResult.SUCCESS:
            solution.update_robot(robot_id, direction, current_turn)
            return True

        return False