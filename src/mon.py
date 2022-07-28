from time import time

import serial
port = "COM5"
ser = serial.Serial(port,115200,timeout=0.01)

import collections

buf = collections.deque(maxlen=256)

picBuf = collections.deque()

protocolInit = bytearray(b"SUK\n")

once = False

tNow = time()
t0 = tNow
t1 = tNow
print("mulai")
while 1:
  incomingBytes = ser.read(2048)
  if len(incomingBytes) != 0:
    open("out.jpg",'ab').write(incomingBytes)

  if not once and tNow -  t0 > 3:
    ser.write(b'tangkap\n')
    once = True
    
  tNow = time()