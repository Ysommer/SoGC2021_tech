from enum import Enum
from typing import List
from controlCenter.algos.InitShell import *
from controlCenter.algos.init_algos.OutAndInByPercentage import *
from controlCenter.algos.OptimizationShell import *

class WishListPackages (Enum):
    SMALL = 0
    MEDIUM = 1
    LARGE = 2
    FARM = 3

    @staticmethod
    def run_small(initShells: List[InitShell], optShells: List[InitShell]):
        for i in range(0, 11):
            initShells.append(InitShell(OutAndInByPercentage, name="", print_info=False, data_bundle={"sync_insertion": False, "percent_to_leave_inside": i*10}))
            initShells.append(InitShell(OutAndInByPercentage, name="", print_info=False, data_bundle={"sync_insertion": False, "percent_to_leave_inside": i*10}))
            initShells.append(InitShell(OutAndInByPercentage, name="", print_info=False, data_bundle={"sync_insertion": False, "percent_to_leave_inside": i*10}))
            initShells.append(InitShell(OutAndInByPercentage, name="", print_info=False, data_bundle={"sync_insertion": False, "percent_to_leave_inside": i*10}))

class WishList:
    def __init__(self, ):
        self.initShells = []
        self.optShells = []


    def optimize_solutions(self, solutionsToOptimizePath: str, number_of_iterations: int = 1):
        pass

    def all_to_all(self, number_of_iterations: int = 1):
        """
        run all init algorithms
        then run number_of_iterations of optimizations
        :return:
        """
        pass

    def zip_and_verify(self):
        pass
