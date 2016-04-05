import binascii
import struct
import numpy as np
from itertools import izip_longest
import itertools


def chunker(seq, size):
    return (seq[pos:pos + size] for pos in xrange(0, len(seq), size))

def writetofile(filename, bitstream):
    with open(filename, "w") as of:
        for x in chunker(bitstream, 8):
            of.write("0x{0:0>2x}, ".format(int(x, 2)))

DATA_OFFSET = 44
bytes = []
with open("sound.wav", "rb") as f:
    f.read(DATA_OFFSET)
    for x in range(3431):#not dynamic, need to read the file in a hex editor first
      bytes += struct.unpack("<HHHHHHHH", f.read(16))

#----------
# 1 bit D/A
#----------
bits = ''
for x in bytes:
  if x > 32768:
    bits += '1'
    continue
  bits += '0'

writetofile("d2a.txt", bits)
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

writetofile("diff.txt", bits)
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

writetofile("os.txt", bits)
#-------------

#-------------
# Oversampling with double difference encoding
#-------------
bits = ''
lenbytes = len(bytes)
for j in range(1,lenbytes):
    lst = bytes[(j - 1) % lenbytes]
    nxt = bytes[(j + 1) % lenbytes]
    if nxt <= bytes[j] <= lst:
        bits += '0110'
    if nxt > bytes[j] < lst:
        bits += '0000'
    if nxt > bytes[j] > lst:
        bits += '1101'
    if nxt < bytes[j] > lst:
        bits += '1111'

writetofile("osdiff.txt", bits)
#-------------
