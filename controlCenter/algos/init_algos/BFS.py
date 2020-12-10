from controlCenter.algos.initAlgo import InitAlgo
from controlCenter.infrastructure.grid import Grid
from controlCenter.defines import directions_to_coords
from controlCenter.infrastructure.robot import Robot
from controlCenter.utils import move_robot_to_dir
import queue
from random import shuffle

class BFS(InitAlgo):

    def __init__(self, instance_name: str, grid: Grid, robots: list, targets: list, max_makespan: int = None, max_sum: int = None, preprocess=None):
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
            if self.grid.get_cell(pos).is_obs():
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

    def step(self) -> int:
        shuffle(self.permutation)
        for i in self.permutation:
            if self.robots[i].robot_arrived:
                continue
            if move_robot_to_dir(self.robots[i], self.grid, self.bfs_list[i][self.progress[i]],
                                 self.current_turn, self.solution):
                self.progress[i] += 1




    def run(self):
        # TODO
        pass
