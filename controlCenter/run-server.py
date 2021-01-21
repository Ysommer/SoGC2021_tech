import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")


from WishList import WishList, WishListPackagesTypes


packageType = WishListPackagesTypes(int(sys.argv[1]))


server_id = int(sys.argv[2])


WishList.farm_instances(packageType, server_id, 11)

