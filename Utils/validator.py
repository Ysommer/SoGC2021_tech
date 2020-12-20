from cgshop2021_pyutils import InstanceDatabase, ZipSolutionIterator, validate, ZipReaderError, InvalidSolutionError, SolutionEncodingError


def validate_solution_zip(instances_zip_path, solution_zip_path):
    idb = InstanceDatabase(instances_zip_path)
    try:
        print("Flag")
        for solution in ZipSolutionIterator(idb)(solution_zip_path):
            print("validating", solution, "...")
            validate(solution)
    except ZipReaderError as zre:
        print("Bad Zip:", zre)
    except InvalidSolutionError as ise:
        print("Bad Solution:", ise)
    except SolutionEncodingError as see:
        print("Bad Solution File:", see)