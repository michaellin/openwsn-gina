The driver is for: 
64bit RZUSBStick 

Note...issues with baud rate on OSX. Need to lower from current value. 
Value found in motetalk.py

 self.ser = serial.Serial(sport, baudrate=57600, timeout=timeout) 

