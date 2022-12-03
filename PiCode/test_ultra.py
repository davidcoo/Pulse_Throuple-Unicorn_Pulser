from ctypes import *
import os
SHM_KEY_RIGHT = 0x1234
SHM_KEY_LEFT = 0x5678
SHM_KEY_FRONT  = 0x1357

so_file = os.path.join(os.path.dirname(__file__), "linkc.so")
linkc = CDLL(so_file)

linkc.set_up(SHM_KEY_RIGHT)
