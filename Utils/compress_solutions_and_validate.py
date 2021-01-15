import glob
import os
import zipfile
from Utils.validator import validate_solution_zip
from os.path import basename
import time


INSTANCES_PATH = "../instances/instances-zip/instances.zip"
SOLUTIONS_PATH = "../Solutions/"
SOLUTION_ZIP_NAME = "../solutions.zip"
SOLUTION_ZIP_NAME_WO_SUFFIX = "../solutions"

SOLUTIONS_ZIP_PATH = ""

MAKESPAN_TAG = "MSPAN"
SUM_TAG = "SUM"


def zipdir(path, ziph):
    for root, dirs, files in os.walk(path):
        for file in files:
            if file == '.DS_Store' or file == 'not_empty.txt':
                continue
            if "SUCCESS" in file:
                ziph.write(os.path.join(root, file), file)


def zip_best(path, ziph):
    for root, dirs, files in os.walk(path):
        min_makespan = -1
        makespan_file = None
        min_sum = -1
        sum_file = None

        for file in files:
            if file == '.DS_Store' or file == 'not_empty.txt':
                continue
            if "SUCCESS" in file:
                left_index = file.find(MAKESPAN_TAG) + len(MAKESPAN_TAG)
                right_index = file[left_index:].find("_") + left_index
                temp_makespan = int(file[left_index:right_index])

                if temp_makespan < min_makespan or min_makespan is -1:
                    min_makespan = temp_makespan
                    makespan_file = file

                left_index = file.find(SUM_TAG) + len(SUM_TAG)
                right_index = file[left_index:].find(".") + left_index
                temp_sum = int(file[left_index:right_index])

                if temp_sum < min_sum or min_sum is -1:
                    min_sum = temp_sum
                    sum_file = file

        if makespan_file is None:
            continue

        ziph.write(os.path.join(root, makespan_file), makespan_file)
        if makespan_file != sum_file:
            ziph.write(os.path.join(root, sum_file), sum_file)


def compress_solutions(solution_paths = SOLUTIONS_PATH, solution_zip_path = SOLUTION_ZIP_NAME):
    zipf = zipfile.ZipFile(solution_zip_path, 'w', zipfile.ZIP_DEFLATED)
    zipdir(solution_paths, zipf)
    zipf.close()


def clean_bad_solutions(solution_paths = SOLUTIONS_PATH):
    for root, dirs, files in os.walk(solution_paths):
        for file in files:
            if file == '.DS_Store' or file == 'not_empty.txt':
                continue
            if "SUCCESS" not in file:
                print("Removing:", root + "/" + file)
                os.remove(root + "/" + file)


def compress_solutions_and_validate():
    clean_bad_solutions()
    compress_solutions()
    validate_solution_zip(INSTANCES_PATH, SOLUTION_ZIP_NAME)


def compress_best_and_send(name="" ,solution_paths = SOLUTIONS_PATH):
    zipf = zipfile.ZipFile(SOLUTION_ZIP_NAME_WO_SUFFIX+name+time.strftime("%Y%m%d-%H%M%S")+".zip", 'w', zipfile.ZIP_DEFLATED)
    zip_best(solution_paths, zipf)
    zipf.close()