import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")

from WishList import WishList

"""
    Variables:
    1: instance_id
    2: number_of_processors - send 1
    3: number_of_inits_per_processor
    4: first_algo
"""

if len(sys.argv) <= 4:
    WishList.farm_instance(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
else:
    WishList.farm_instance(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))