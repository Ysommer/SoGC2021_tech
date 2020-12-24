import json
from infrastructure.solGrid import SolGrid

class BasicTests:
    def run_test(self):
        pass

    def run_solGrid_test(self):
        Instance_name = "the_king_94"
        Sol_name = "the_king_94_OutAndInBFS_default_SUCCESS_MSPAN3038_SUM4339.json"
        path = "../solutions/"+Instance_name+"/"+Sol_name
        file = open(path, "r")
        sol_json = json.load(file)
        SolGrid()

