import serial

ser = serial.Serial('/dev/ttyUSB0')
while True:
  print ser.readline(),

