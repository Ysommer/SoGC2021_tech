import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")


from WishList import WishList, WishListPackagesTypes
from Utils.compress_solutions_and_validate import compress_best_and_send


packageType = WishListPackagesTypes(int(sys.argv[1]))


server_id = int(sys.argv[1])


WishList.farm_instances(packageType, server_id, 11)
compress_best_and_send(packageType.name)
