from algos.initAlgo import InitAlgo
from infrastructure.grid import Grid
from defines import directions_to_coords, SolutionResult
from infrastructure.robot import Robot
from infrastructure.cell import *
from utils import *
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

    def calc_bfs(self, i: int, blocked: list = None) -> list:
        if blocked is None:
            blocked = []
        parents = {self.robots[i].pos: None}
        # visited = [self.robots[i].pos]
        q = queue.Queue()
        q.put(self.robots[i].pos)

        def legal_step(pos: (int, int)) -> bool:
            next_cell = self.grid.get_cell(pos)
            if next_cell.is_obs() or next_cell.has_robot_on_target():
                return False
            return -3 <= pos[0] <= self.grid.size + 2 and -3 <= pos[1] <= self.grid.size + 2

        def construct_path(parents: dict, pos: (int, int))-> list:
            path = []
            while parents[pos] is not None:
                path.append(parents[pos])
                pos = sub_tuples(pos, directions_to_coords[parents[pos]])

            return path[::-1]  # return reversed path

        while not q.empty():
            pos = q.get()
            if pos == tuple(self.targets[i]):
                return construct_path(parents, pos)
            for direction in directions_to_coords:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if next_pos not in parents and pos not in blocked and legal_step(next_pos):
                    q.put(next_pos)
                    # visited.append(next_pos)
                    parents[next_pos] = direction

        return []

    def unclog(self, num_to_recalc: int):
        recalced = []
        shuffle(self.permutation)
        for i in self.permutation:
            if self.robots[i].robot_arrived():
                continue
            blocked = []
            pos = self.robots[i].pos
            for direction in directions_to_coords:
                neighbor_pos = sum_tuples(pos, directions_to_coords[direction])
                neighbor = self.grid.get_cell(neighbor_pos).has_robot()
                if neighbor is not None and neighbor not in recalced:
                    blocked.append(neighbor_pos)
            new_bfs = self.calc_bfs(i, blocked)
            if len(new_bfs) > 0:
                self.bfs_list[i] = new_bfs
                self.progress[i] = 0
                recalced.append(i)
                if len(recalced) >= num_to_recalc:
                    return len(recalced)
        return len(recalced)



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
            if self.robots[i].robot_arrived():
                continue
            if InitAlgo.move_robot_to_dir(i, self.grid, self.bfs_list[i][self.progress[i]],
                                          self.current_turn, self.solution):
                self.progress[i] += 1
                moved += 1
        return moved

    def run(self):
        # i = -1  # REMOVE
        turns_stuck = 0
        while True:
            # i += 1  # REMOVE
            # print(i)  # REMOVE
            if self.current_sum > self.max_sum:
                self.solution.put_result(SolutionResult.EXCEEDED_MAX_SUM, self.current_turn, self.current_sum)
                self.solution.print()
                return self.solution

            if self.current_turn > self.max_makespan:
                self.solution.put_result(SolutionResult.EXCEEDED_MAX_MAKESPAN, self.current_turn, self.current_sum)
                self.solution.print()
                return self.solution

            if self.grid.solution_found():
                self.solution.put_result(SolutionResult.SUCCESS, self.current_turn, self.current_sum)
                self.solution.print()
                return self.solution

            self.solution.out["steps"].append({})

            last_turn_sum = self.step()

            if last_turn_sum == 0:
                turns_stuck += 1
                if turns_stuck >= 2:
                    self.solution.put_result(SolutionResult.STUCK, self.current_turn, self.current_sum)
                    self.solution.print()
                    return self.solution
                robots_remaining = len(self.robots) - self.grid.numOfRobotsArrived
                print("remain: ", robots_remaining)
                NUM_TO_UNCLOG = robots_remaining//2
                recalced = self.unclog(NUM_TO_UNCLOG)
                print("recalced: ", recalced)
                continue

            turns_stuck = 0
            self.current_turn += 1
            self.current_sum += last_turn_sum
