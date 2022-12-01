from ctypes import *
import os 
so_file = os.path.join(os.path.dirname(__file__), "my_functions.so")
my_functions = CDLL(so_file)


print(type(my_functions))

print(my_functions.square(10))

print(my_functions.square(5))