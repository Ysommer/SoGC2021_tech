from controlCenter.algos.initAlgo import InitAlgo
from controlCenter.infrastructure.grid import Grid
from controlCenter.defines import directions_to_coords
from controlCenter.infrastructure.robot import Robot
from controlCenter.utils import move_robot_to_dir
from controlCenter.infrastructure.cell import Cell
import queue
from typing import List
from random import shuffle, randint


class BFS(InitAlgo):

    def __init__(self, instance_name: str, grid: Grid, robots: List[Robot], targets: list, max_makespan: int = None, max_sum: int = None, preprocess=None):
        super().__init__(instance_name, grid, robots, targets, max_makespan, max_sum, preprocess)

        self.bfs_list = []
        for i in range(len(robots)):
            self.bfs_list.append(self.calc_bfs(i))
            assert len(self.bfs_list[i]) > 0

        self.permutation = [i for i in range(len(self.robots))]
        self.progress = [0 for i in range(len(self.robots))]

    def calc_bfs(self, i: int) -> list:
        parents = {self.robots[i].pos: None}
        # visited = [self.robots[i].pos]
        q = queue.Queue()
        q.put(self.robots[i].pos)

        def legal_step(pos: (int, int)) -> bool:
            if self.grid.get_cell(pos).is_obs or self.grid.get_cell(pos).has_robot_on_target:
                return False
            return -1 <= pos[0] <= self.grid.size+1 and -1 <= pos[1] <= self.grid.size

        def construct_path(parents: dict, pos: (int, int))-> list:
            path = []
            while parents[pos] is not None:
                path.append(parents[pos])
                pos -= parents[pos]

            return path[::-1]  # return reversed path

        while not q.empty():
            pos = q.get()
            if pos == self.targets[i]:
                return construct_path(parents, pos)
            for direction in directions_to_coords:
                next_pos = pos + directions_to_coords[direction]
                if next_pos not in parents and legal_step(pos):
                    q.put(next_pos)
                    # visited.append(next_pos)
                    parents[next_pos] = direction

        return []

    def divert(self, num_to_divert: int):
        odd_round = True  # used to randomize diversion direction
        diverted = []

        def direction_swapper(arg: (str, bool)) -> (str, str):
            swap = {
                ("W", False): ("S", "N"),
                ("E", False): ("N", "S"),
                ("N", False): ("W", "E"),
                ("S", False): ("E", "W"),
                ("W", True): ("N", "S"),
                ("E", True): ("S", "N"),
                ("N", True): ("E", "W"),
                ("S", True): ("W", "E"),
            }
            return swap.get(arg)

        while len(diverted) < num_to_divert:
            i = randint(range(len(self.robots)))
            if i not in diverted and not self.robots[i].robot_arrived:
                diverted.append(i)
                current_direction = self.bfs_list[i][self.progress[i]]
                swap = direction_swapper((current_direction, odd_round))
                pos = self.robots[i]
                for direction in swap:
                    pass

    def step(self) -> int:
        moved = 0
        shuffle(self.permutation)
        for i in self.permutation:
            if self.robots[i].robot_arrived:
                continue
            if move_robot_to_dir(self.robots[i], self.grid, self.bfs_list[i][self.progress[i]],
                                 self.current_turn, self.solution):
                self.progress[i] += 1
                moved += 1
        return moved

    def run(self):
        while True:
            if self.current_sum > self.max_sum:
                self.solution.result = SolutionResult.EXCEEDED_MAX_SUM
                return self.solution

            if self.current_turn > self.max_makespan:
                self.solution.result = SolutionResult.EXCEEDED_MAX_MAKESPAN
                return self.solution

            if self.grid.solution_found():
                self.solution.result = SolutionResult.SUCCESS
                return self.solution

            self.solution.out["steps"].append({})

            last_turn_sum = self.step()

            if last_turn_sum == 0:
                self.solution.result = SolutionResult.STUCK
                return self.solution

            self.current_turn += 1
            self.current_sum += last_turn_sum
