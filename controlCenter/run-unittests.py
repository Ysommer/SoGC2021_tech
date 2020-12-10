from unittests.basic_tests import *
from unittests.test_sorts import *

tests = [
    TestSorts()
]

for t in tests:
    t.run()