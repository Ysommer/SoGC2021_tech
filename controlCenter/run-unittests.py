from unittests.basic_tests import *
from unittests.test_sorts import *
from unittests.GeneratorUnitTests import *

tests = [
    TestSorts(),
    GeneratorUnitTests()
]

for t in tests:
    t.run()