from infrastructure.cell import Cell
from infrastructure.robot import Robot
from defines import *
from utils import sum_tuples


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
        self.outBoundaries = {}

        self.__set_obstacles()
        self.__set_targets()
        self.__set_robots()
        global directions_to_coords

    def get_cell(self, pos: (int, int)) -> Cell:
        if (0 <= pos[0] < self.size) and (0 <= pos[1] < self.size):
            if self.grid[pos[0]][pos[1]] is None:
                self.grid[pos[0]][pos[1]] = Cell(pos)
            return self.grid[pos[0]][pos[1]]

        if pos not in self.outBoundaries:
            self.outBoundaries[pos] = Cell(pos)

        return self.outBoundaries[pos]

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
