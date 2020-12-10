from dataCollection.preprocess import Preprocess
from unittests.basic_tests import BasicTests
from infrastructure.robot import Robot
from random import Random

class TestSorts:
    def __init__(self):
        self.cases = 1000
        self.n = 100
        self.robots = []
        self.rnd = Random()
        for i in range(self.cases):
            pos = (int(Random.uniform(self.rnd, 0, self.n)), int(Random.uniform(self.rnd, 0, self.n)))
            target_pos = (int(Random.uniform(self.rnd, 0, self.n)), int(Random.uniform(self.rnd, 0, self.n)))
            self.robots.append(Robot(i,pos,target_pos))

        self.preprocessor = Preprocess(None, self.robots)

    def run(self):
        # Sort by robots coordinates
        self.test_R_x()
        self.test_R_X()
        self.test_R_y()
        self.test_R_Y()
        self.test_R_x_y()
        self.test_R_x_Y()
        self.test_R_X_y()
        self.test_R_X_Y()
        self.test_R_y_x()
        self.test_R_y_X()
        self.test_R_Y_x()
        self.test_R_Y_X()

    def test_R_x(self):
        res = self.preprocessor.sort_R_x()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i+1]]
            if current_robot.pos[0] > next_robot.pos[0]:
                print("Error! i: " + str(i) +
                      ". current_robot.x: "+ str(current_robot.pos[0]) +
                      ". next_robot.x: "+ str(next_robot.pos[0]))
                assert 0

    def test_R_X(self):
        res = self.preprocessor.sort_R_X()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[0] < next_robot.pos[0]:
                print("Error! i: " + str(i) +
                      ". current_robot.x: " + str(current_robot.pos[0]) +
                      ". next_robot.x: " + str(next_robot.pos[0]))
                assert 0

    def test_R_y(self):
        res = self.preprocessor.sort_R_y()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[1] > next_robot.pos[1]:
                print("Error! i: " + str(i) +
                      ". current_robot.y: " + str(current_robot.pos[1]) +
                      ". next_robot.y: " + str(next_robot.pos[1]))
                assert 0

    def test_R_Y(self):
        res = self.preprocessor.sort_R_Y()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[1] < next_robot.pos[1]:
                print("Error! i: " + str(i) +
                      ". current_robot.y: " + str(current_robot.pos[1]) +
                      ". next_robot.y: " + str(next_robot.pos[1]))
                assert 0

    def test_R_x_y(self):
        res = self.preprocessor.sort_R_x_y()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[0] > next_robot.pos[0] or \
                (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] > next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_x_Y(self):
        res = self.preprocessor.sort_R_x_Y()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[0] > next_robot.pos[0] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] < next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_X_y(self):
        res = self.preprocessor.sort_R_X_y()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[0] < next_robot.pos[0] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] > next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_X_Y(self):
        res = self.preprocessor.sort_R_X_Y()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[0] < next_robot.pos[0] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] < next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_y_x(self):
        res = self.preprocessor.sort_R_y_x()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[1] > next_robot.pos[1] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] > next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_Y_x(self):
        res = self.preprocessor.sort_R_Y_x()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[1] < next_robot.pos[1] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] > next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_y_X(self):
        res = self.preprocessor.sort_R_y_X()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[1] > next_robot.pos[1] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] < next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0

    def test_R_Y_X(self):
        res = self.preprocessor.sort_R_Y_X()
        for i in range(len(res) - 1):
            current_robot = self.robots[res[i]]
            next_robot = self.robots[res[i + 1]]
            if current_robot.pos[1] < next_robot.pos[1] or \
                    (current_robot.pos[0] == next_robot.pos[0] and current_robot.pos[1] < next_robot.pos[1]):
                print("Error! i: " + str(i) +
                      ". current_robot.pos: " + str(current_robot.pos) +
                      ". next_robot.pos: " + str(next_robot.pos))
                assert 0