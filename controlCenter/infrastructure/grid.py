from infrastructure.cell import Cell
from infrastructure.robot import Robot
from defines import *
from utils import sum_tuples


class BFSCounters:
    def __init__(self, last_bfs_counter=0, last_bfs_parent=None, last_configured_dist=-1):
        # BFS support
        self.last_bft_counter = last_bfs_counter
        self.last_bfs_parent = last_bfs_counter

        # Distance support
        self.last_configured_dist = last_configured_dist


class Grid:
    def __init__(self, size: int, robots: list, obstacles: list):
        self.size = size

        # Copy robots to avoid different Grid instances using the same list
        self.robots = robots
        self.obstacles = obstacles
        self.numOfRobots = len(robots)
        self.numOfRobotsArrived = 0
        for r in robots:
            if r.robot_arrived():
                self.numOfRobotsArrived += 1

        self.grid = [[None for i in range(self.size)] for j in range(self.size)]
        self.bfs_grid = None

        self.out_boundaries = {}
        self.grid_out_boundaries = {}

        self.__set_obstacles()
        self.__set_targets()
        self.__set_robots()

        self.bfs_counter = 0
        global directions_to_coords

    def get_cell(self, pos: (int, int)) -> Cell:
        if (0 <= pos[0] < self.size) and (0 <= pos[1] < self.size):
            if self.grid[pos[0]][pos[1]] is None:
                self.grid[pos[0]][pos[1]] = Cell(pos)
            return self.grid[pos[0]][pos[1]]

        if pos not in self.out_boundaries:
            self.out_boundaries[pos] = Cell(pos)

        return self.out_boundaries[pos]

    def check_if_cell_is_free(self, pos: (int, int)):
        """
        Do not take tail into consideration
        """
        if (0 <= pos[0] < self.size) and (0 <= pos[1] < self.size):
            if self.grid[pos[0]][pos[1]] is None:
                return True
            return self.grid[pos[0]][pos[1]].enter_cell(robot_id=-1, direction=None, current_turn=-1, advance=False) == EnterCellResult.SUCCESS

        if pos not in self.out_boundaries:
            return True

        return self.out_boundaries[pos[0], pos[1]].enter_cell(robot_id=-1, direction=None, current_turn=-1, advance=False) == EnterCellResult.SUCCESS

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

    def start_bfs(self):
        if self.bfs_grid is None:
            self.bfs_grid = [[BFSCounters() for i in range(self.size)] for j in range(self.size)]
        self.bfs_counter += 1

    def get_cell_for_bfs(self, pos):
        if (0 <= pos[0] < self.size) and (0 <= pos[1] < self.size):
            return self.bfs_grid[pos[0]][pos[1]]

        if pos not in self.out_boundaries:
            self.out_boundaries[pos] = BFSCounters(last_bfs_counter=self.bfs_counter, last_configured_dist=self.bfs_counter)

        return self.out_boundaries[pos]


    def __set_robots(self):
        for r in range(len(self.robots)):
            robot = self.robots[r]
            self.get_cell(robot.pos).place_robot(r)

    def __set_targets(self):
        for t in range(len(self.robots)):
            self.get_cell(self.robots[t].target_pos).place_target(t)

    def __set_obstacles(self):
        for o in range(len(self.obstacles)):
            pos = (self.obstacles[o][0], self.obstacles[o][1])
            self.get_cell(pos).place_obstacle()
