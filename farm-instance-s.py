import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")

from WishList import WishList

"""
    Variables:
    1: instance_id
    2: number_of_inits
    3: first_algo
    4: opt_iters
"""

WishList.farm_instance_s(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))