import os
import zipfile
from Utils.validator import validate_solution_zip
from os.path import basename


INSTANCES_PATH = "../instances/instances-zip/instances.zip"
SOLUTIONS_PATH = "../Solutions/"
SOLUTION_ZIP_NAME = "solutions.zip"
SOLUTIONS_ZIP_PATH = ""


def zipdir(path, ziph):
    for root, dirs, files in os.walk(path):
        for file in files:
            if "SUCCESS" in file:
                ziph.write(os.path.join(root, file))


def compress_solutions(solution_paths = SOLUTIONS_PATH, solution_zip_path = SOLUTIONS_PATH):
    zipf = zipfile.ZipFile(SOLUTION_ZIP_NAME, 'w', zipfile.ZIP_DEFLATED)
    zipdir(SOLUTIONS_PATH, zipf)
    zipf.close()


def clean_bad_solutions(solution_paths = SOLUTIONS_PATH):
    for root, dirs, files in os.walk(solution_paths):
        for file in files:
            if "SUCCESS" not in file:
                print("Removing:", root + "/" + file)
                os.remove(root + "/" + file)


def compress_solutions_and_validate():
    clean_bad_solutions()
    compress_solutions()
    validate_solution_zip(SOLUTIONS_PATH, INSTANCES_PATH)