from .cell import Cell

class Grid:
    def __init__(self, size: int, boundaries_percentage: int = 30):
        self.size = size
        self.grid = [[None for i in range(self.size)] for j in range(self.size)]

    def getCell(self, pos: (int, int)):
        if self.grid[pos[0]][pos[1]] is None:
            self.grid[pos[0]][pos[1]] = Cell(pos)

        return self.grid[pos[0]][pos[1]]