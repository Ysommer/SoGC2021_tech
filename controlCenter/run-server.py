from WishList import WishList, WishListPackagesTypes
import sys

packageType = WishListPackagesTypes(sys.argv[1])
server_id = int(sys.argv[2])

WishList.farm_instances(packageType, server_id)