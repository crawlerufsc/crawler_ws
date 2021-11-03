import serial
import time
import string 
import pynmea
while True: 
    port='/dev/ttySO'
    ser=serial.Serial(port,baudrate=9600,timeout=0.5)
    dataout =pynmea.NMEAStreamReader()
    newdata=ser.readline()
    if newdata[0:6]=='$GPRMC':
        newmsg=pynmea.parse(newdata)
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps='Latitude=' +str(lat) + 'and Longitude=' +str(lng)
        print(gps)
