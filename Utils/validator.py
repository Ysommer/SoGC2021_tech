from cgshop2021_pyutils import InstanceDatabase, ZipSolutionIterator, validate, ZipReaderError, InvalidSolutionError, SolutionEncodingError
idb = InstanceDatabase("..\instances\instances-zip\instance.zip")
try:
    for solution in ZipSolutionIterator(idb)("..\solutions\solutions-zip\solution.zip"):
        validate(solution)
except ZipReaderError as zre:
    print("Bad Zip:", zre)
except InvalidSolutionError as ise:
    print("Bad Solution:", ise)
except SolutionEncodingError as see:
    print("Bad Solution File:", see)