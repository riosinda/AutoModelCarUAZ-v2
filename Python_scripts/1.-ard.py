import serial,time

serArduino=serial.Serial("COM5",115200,timeout=1)
time.sleep(1)

while True:
	serArduino.write(b'70*25\n')

ser.close()