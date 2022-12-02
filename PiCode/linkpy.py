from ctypes import *
import os

so_file = os.path.join(os.path.dirname(__file__), "linkc.so")

linkc = CDLL(so_file)


linkc.write_vals(b"123456789abcdef")