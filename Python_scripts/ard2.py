import serial,time

serArduino = serial.Serial("/dev/ttyUSB1",115200,timeout=1)
time.sleep(1)

while True:
    serArduino.write(b'88*13\n')
    
    
serArduino.close()

