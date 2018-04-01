import numpy as np


filename = 'colliders.csv'

# TODO: read lat0, lon0 from colliders into floating point values
# Read first line into var
# with open(filename) as f:
#     x,y = f.readline().rstrip().split(',')

with open(filename) as f:
    x,y = f.readline().split(',')
_,x= x.split(' ')
_,y = y.lstrip().split(' ')

x = float(x)
print(x)

# x,y = np.fromstring(line, dtype='Float64', count=2, sep=',')

print(y)

print(type(x))
# lat0, lon0 = np.loadtxt(first, delimiter=',', dtype='Float64')

