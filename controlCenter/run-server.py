from WishList import WishList, WishListPackagesTypes
import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")

packageType = WishListPackagesTypes(sys.argv[1])
server_id = int(sys.argv[2])

WishList.farm_instances(packageType, server_id)