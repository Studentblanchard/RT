import binascii
import struct
import numpy as np
import matplotlib.pyplot as plt
from itertools import izip_longest
import itertools

DATA_OFFSET = 44
bytes = []
with open("sound.wav", "rb") as f:
    f.read(DATA_OFFSET)
    for x in range(3431):
      bytes += struct.unpack("<HHHHHHHH", f.read(16))

#This will print each of the values from the data
#print '\n'.join(map(str, bytes))

#Check out the plot of the sample if you wish
#line, = plt.plot(range(len(bytes)),bytes,linewidth=2)
#plt.show()

#----------
# 1 bit D/A
#----------
bits = ''
for x in bytes:
  if x > 32768:
    bits += '1'
    continue
  bits += '0'
if raw_input("u want 1 bit D/A? ") == 'yes':
  print bits
#----------


#--------------------
# Difference Encoding
#--------------------
bits = ''
for i in range(1,len(bytes)):
  if bytes[i-1] < bytes[i]:
    bits += '1'
    continue
  bits += '0'
if raw_input("how about diff encoded hmm? ") == 'yes':
  print bits
#--------------------


#-------------
# Oversampling
#-------------
limits = np.linspace(0,65536, num=6)
bits = ''
y = ['0000','0010','0101','1101','1111']
for x in bytes:
  for z in range(0,5)[::-1]:
    if x >= limits[z]:
      bits += y[z]
      break
if raw_input("last but not least oversampling? ") == 'yes':
  print bits
#-------------
