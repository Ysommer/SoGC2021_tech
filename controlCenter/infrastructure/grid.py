import queue
from copy import copy, deepcopy

from infrastructure.cell import Cell
from infrastructure.robot import Robot
from defines import *
from utils import sum_tuples
from infrastructure.BFSCounterCell import BFSCounterCell


class Grid:
    def __init__(self, size: int, robots: list, obstacles: list):
        self.size = size

        # Copy robots to avoid different Grid instances using the same list
        self.robots = robots
        self.obstacles = set()

        for i in obstacles:
            self.obstacles.add(i)

        self.numOfRobots = len(robots)
        self.numOfRobotsArrived = 0
        for r in robots:
            if r.robot_arrived():
                self.numOfRobotsArrived += 1

        self.grid = {}
        self.bfs_grid = {}

        self.__set_obstacles()
        self.__set_targets()
        self.__set_robots()

        self.bfs_counter = 0

    def is_completely_clean_cell(self, pos):
        if pos not in self.grid:
            return True
        cell = self.get_cell(pos)
        return cell.is_empty() and cell.target_id == -1

    def is_obs(self, pos) -> bool:
        return pos in self.obstacles

    def has_robot(self, pos):
        return pos in self.grid and self.get_cell(pos).has_robot()

    def get_robot_id_by_pos(self, pos):
        return self.get_cell(pos).get_robot()

    def has_robot_on_target(self, pos):
        return pos in self.grid and self.get_cell(pos).has_robot_on_target()

    def get_cell(self, pos: (int, int)) -> Cell:
        if pos not in self.grid:
            self.grid[pos] = Cell(pos)

        return self.grid[pos]

    def check_if_cell_is_free(self, pos: (int, int)):
        """
        Do not take tail into consideration
        """
        if pos not in self.grid:
            return True

        return self.grid[pos].is_empty()

    def move_robot(self, robot_id: int, direction: str, current_turn: int) -> EnterCellResult:
        robot = self.robots[robot_id]
        old_pos = robot.pos
        old_cell = self.get_cell(old_pos)
        desired_pos = sum_tuples(robot.pos, directions_to_coords[direction])

        desired_cell = self.get_cell(desired_pos)

        enter_result = desired_cell.enter_cell(robot_id, direction, current_turn)
        if enter_result != EnterCellResult.SUCCESS:
            return enter_result

        old_cell.exit_cell(direction, current_turn)
        # Checking if robot leaves target
        if robot.robot_arrived():
            self.numOfRobotsArrived -= 1

        robot.step(direction)
        # Checking if robot arrived
        if robot.robot_arrived():
            self.numOfRobotsArrived += 1

        return enter_result

    def solution_found(self) -> bool:
        return self.numOfRobotsArrived == self.numOfRobots

    def start_bfs(self, started_positions: queue.Queue, is_a_star: bool = False):
        assert started_positions is not None, "start_bfs: started_positions is None"
        self.bfs_counter += 1

        started_positions.put(None)
        pos = started_positions.get()
        while pos is not None:
            if is_a_star:
                self.check_cell_for_a_star(pos=pos, parent="", g_value=0)
            else:
                self.check_cell_for_bfs(pos=pos, parent="", dist=0)
            started_positions.put(pos)
            pos = started_positions.get()

    def get_cell_for_bfs(self, pos) -> BFSCounterCell:
        if pos not in self.bfs_grid:
            self.bfs_grid[pos] = BFSCounterCell(last_bfs_counter=0, last_configured_dist=-1)

        return self.bfs_grid[pos]

    def check_cell_for_bfs(self, pos, parent: str = "", dist=-1, to_update=True) -> bool:
        return self.get_cell_for_bfs(pos).check_move(new_bfs_counter=self.bfs_counter, parent=parent, dist=dist, to_update=to_update)

    def check_cell_for_a_star(self, pos, parent: str, g_value: int) -> bool:
        return self.get_cell_for_bfs(pos).check_move_a_star(new_bfs_counter=self.bfs_counter, parent=parent, g_value=g_value)

    def check_if_cell_is_open(self, pos, g_value):
        return self.get_cell_for_bfs(pos).check_if_open(new_bfs_counter=self.bfs_counter, g_value=g_value)

    def get_cell_parent(self, pos):
        return self.get_cell_for_bfs(pos).get_parent(self.bfs_counter)

    def get_cell_distance(self, pos, get_old_configure=True):
        return self.get_cell_for_bfs(pos).get_distance(self.bfs_counter, get_old_configure)

    def get_copy_bfs_map(self, get_old_configure=True) -> dict:
        out = {}
        for pos in self.bfs_grid:
            out[pos] = self.get_cell_distance(pos, get_old_configure)

        return out

    def __set_robots(self):
        for r in range(len(self.robots)):
            robot = self.robots[r]
            self.get_cell(robot.pos).place_robot(r)

    def __set_targets(self):
        for t in range(len(self.robots)):
            self.get_cell(self.robots[t].target_pos).place_target(t)

    def __set_obstacles(self):
        for obs in self.obstacles:
            pos = (obs[0], obs[1])
            self.get_cell(pos).place_obstacle()
