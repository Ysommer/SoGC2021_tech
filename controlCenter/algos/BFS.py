from .initAlgo import InitAlgo
from ..infrastructure.grid import Grid


class BFS(InitAlgo):

    def __init__(self, instance_name: str, grid: Grid, robots: list, targets: list, max_makespan: int = None, max_sum: int = None, preprocess=None):
        super().__init__(instance_name,grid, robots, targets, max_makespan, max_sum, preprocess)

    def step(self):
        # TODO
        pass

    def run(self):
        # TODO
        pass
