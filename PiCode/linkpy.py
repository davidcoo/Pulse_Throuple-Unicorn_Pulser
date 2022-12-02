from ctypes import *
import os

so_file = os.path.join(os.path.dirname(__file__), "linkc.so")

linkc = CDLL(so_file)

linkc.set_up()

print(linkc.test_vals())


#linkc.write_vals(b"123456789abcdef")

import random
#while (1):
linkc.write_vals(bytes(str(random.randint(1,20)).encode("ascii")))
