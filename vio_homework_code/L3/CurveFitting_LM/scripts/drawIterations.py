#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) < 2:
    print('Usage: scipt + data_file')
    sys.exit(0)
    
file = sys.argv[1]
data = []
with open(file, 'r') as f:
    lines = f.readlines()
    for line in lines:
        #print float(line)
        data.append(float(line)/1000.)

x = [float(s) for s in range(len(data))]
plt.plot(x, data, color='red', linewidth=1, linestyle='-', marker='o')

plt.xlabel('Iteration Time')
plt.ylabel('Lambda Value (x 10^3)')
plt.title('Lambda Changes with Iteration Time')
plt.show()
