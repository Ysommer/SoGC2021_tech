from .algo import Algo
from ..infrastructure.grid import Grid


class InitAlgo(Algo):

    def __init__(self, instance_name: str, grid: Grid, robots: list, targets: list, max_makespan: int = None, max_sum: int = None, preprocess=None)::