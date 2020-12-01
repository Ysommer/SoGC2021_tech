import abc
from ..solution import solution


class Algo(abc):

    def __init__(self, grid, robots: list, targets: list, max_makespan: int = None, max_sum: int = None, preprocess= None):
        self.grid = grid
        self.robots = robots
        self.targets = targets
        self.max_makespan = max_makespan
        self.max_sum = max_sum
        self.preprocess = preprocess
        self.current_turn = 0
        self.current_sum = 0
        self.solution = solution()

    @abc.abstractmethod
    def step(self):
        """
        defines a simultaneous step in the algorithm and updates solution.
        """
        ...

    @abc.abstractmethod
    def run(self) -> solution:
        """
        defines the execution of the algorithm.
        should look something like this:
        while(True):
            if self.max_makespan is not None and self.max_makespan <= self.current_turn:
                solution[Result] = SolutionResult.EXCEEDED_MAX_MAKESPAN
                break
            if self.max_sum is not None and self.max_sum <= self.current_sum:
                solution[Result] = SolutionResult.EXCEEDED_MAX_SUM
            self.step()
            if solution.solution_found():
                solution[Result] = SUCCESS
            self.current_turn += 1

        return solution
        """
        ...

