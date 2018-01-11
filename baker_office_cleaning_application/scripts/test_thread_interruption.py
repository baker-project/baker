#!/usr/bin/env python

import thread
import time

def func(x):
    y = x
    for i in range(0, 100):
        print("xf=", y[0], "  id=", id(y))
        time.sleep(0.01)
    
if __name__ == '__main__':
    x = [1]
    thread.start_new_thread(func, (x,))
    time.sleep(0.1)
    x[0] = 2
    time.sleep(1)
    print("x=", x[0], "  id=", id(x))
    