#!/usr/bin/python
try:
  import termios
  termios.B921600 = 0x1007
except:
  pass

import serial
from struct import calcsize, unpack_from
import sys
import time
import ginacmd

def threshold(val, minv, maxv):
  if val < minv: val = minv
  if val > maxv: val = maxv
  return val

def splitbytes(val):
  val_dh = (val >> 8) & 0xff
  val_dl = val & 0xff
  return (val_dl, val_dh)

def setpwm(LEDg, LEDo, mot1, mot0, srv1, srv2, srv3): # generates serial output string
  # check imputs and correct input if necassary
  if LEDg <= 0: LEDg = 0
  else: LEDg = 1

  if LEDo <= 0: LEDo = 0
  else: LEDo = 1

  mot0 = threshold(mot0, 0, 800)
  mot1 = threshold(mot1, 0, 800)
  srv1 = threshold(srv1, -1000, 1000)
  srv2 = threshold(srv2, -1000, 1000)
  srv3 = threshold(srv3, -1000, 1000)

  # calculate bytes
  (mot0_dl, mot0_dh) = splitbytes(mot0)
  (mot1_dl, mot1_dh) = splitbytes(mot1)

  (srv1_dl, srv1_dh) = splitbytes(srv1)
  (srv2_dl, srv2_dh) = splitbytes(srv2)
  srv3 = (srv3<<4)+((1-LEDg)<<1)+(1-LEDo)
  (srv3_dl, srv3_dh) = splitbytes(srv3)
  return [mot1_dl, mot1_dh, mot0_dl, mot0_dh, srv1_dl, srv1_dh, srv2_dl, srv2_dh, srv3_dl, srv3_dh]

def packetify(arr):
  arr = flatten(arr)
  l = [len(arr)]
  l.extend(arr)
  l.append((~sum(arr)) & 0xff) #CRC
  return l

def flatten(l):
  res = []
  for r in l:
    if isinstance(r, list):
      res.extend(flatten(r))
    else:
      res.append(r)
  return res

def crc(bytes):
  c = 0
  for b in bytes:
    c = c ^ b
    for y in range(8):
      if c & 0x1:
        c = c >> 1 ^ 0x8408
      else:
        c = c >> 1
  return c

def crcint(ints):
  bytes = []
  for i in ints:
    bytes.extend(splitbytes(i))
  return crc(bytes)

class motetalk:
  ser = None
  last = ""
  arr = []
  ts = 0

  header = ""
  fmt = ''

  def __repr__(self): 
    return repr(self.ts) + " " + " ".join(map(str, self.arr))
	
  def __init__(self, fmtstring="xH", head="n", sport="COM5", timeout=None, brate=921600):
    try:
      self.ser = serial.Serial(sport, baudrate=brate, timeout=timeout)   #sets COM No. and Baudrate
    except serial.serialutil.SerialException:
      print "No mote connected."
    else:
      if ((fmtstring[0] != "@") or
          (fmtstring[0] != "!") or
          (fmtstring[0] != ">") or
          (fmtstring[0] != "<") or
          (fmtstring[0] != "=")):
        self.fmt = fmtstring
      else:
        self.fmt = "=" + fmtstring
      self.header = "ts " + head
      self.ser.flushInput()
      self.ser.flushOutput()

  def newline(self):
    self.ser.flushInput()
    self.ser.flushOutput()
    numstar = 0
    while numstar < 3: # counts 3 stars before transmission starts
      m = self.ser.read(1)   #read 1st byte
      if m == '*':
        numstar = numstar + 1
      else:
        numstar = 0
		
  def flush(self):
	self.ser.flushInput()
	
  def nextline(self, parse=True):
    if (self.ser == None):
      return False, "Disconnected.", 0
      
    pkg = ''
    numstar = 0
    t = 0
    try:
      while (len(pkg) < 255) and (numstar < 3): # search for 3 stars in a row
        s = self.ser.read(1)
        if (s == ''):
          return None, t, 0
        if not t:
          t = time.time()
        pkg = pkg + s

        if s == '*':
          numstar = numstar + 1
        else:
          numstar = 0

      if (numstar == 3):
        if parse and (len(pkg) >= calcsize(self.fmt)): # 3 stars => write package in file
          self.last = pkg
          self.ts = t
          self.arr = list(unpack_from('>HHHHHHHHBBBBBBH', pkg))
          return self.arr, t, crc(map(ord,pkg[:-5]))
        elif parse:
          return None, t, 0
        else:
          return pkg, t, crc(map(ord,pkg[:-5]))

      else: # Packet not received
        return None, t, 0

    except:  #stops program
      return False, sys.exc_info()[:2], 0

  def sendraw(self, bytes, debug=0): # sends raw bytes to the serial port
    bytes = flatten(bytes)

    # build output string
    ser_str = "".join(map(chr, bytes))
    self.ser.write(ser_str)
    if debug:
      sys.stderr.write(repr([time.time(), bytes]) + "\n")
    time.sleep(0.001)

  def send(self, bytes, debug=0): # generates serial output string
    # calculate checksum
    bytes = flatten(bytes)

    chksum = ~sum(bytes) & 0xff
    bytes.insert(0, len(bytes))
    bytes.insert(0, 0x80)
    bytes.insert(0, 0x80)
    bytes.insert(0, 0x80)
    bytes.append(chksum)

    # build output string
    ser_str = "".join(map(chr, bytes))
    self.ser.write(ser_str)
    if debug:
      sys.stderr.write(repr([time.time(), bytes]) + "\n")
    time.sleep(0.001)

  def sendbase(self, bytes, debug=0): # generates serial output string
    self.send(bytes, debug)

  def loadheli(self, bytes, debug=0): # generates serial output string
    self.sendbase(cmd.buildpkt(packetify(bytes)), debug)

  def sendheli(self, bytes, debug=0): # generates serial output string
    self.sendbase(cmd.sendpkt(packetify(bytes)), debug)

  def end(self):
    if (self.ser):
      self.ser.close()

if __name__ == '__main__':
  from optparse import OptionParser

  cmd = ginacmd.ginacmd("commands/commands.h")

  rocketheader = "n p q r ti ta x y z a lqi rssi"
  rocketfmtstr = "xx H HHHHH xx HHHH xxxx BB"
  heliheader =    "n p q r ti ta X1 x X2 y X3 z ir0 ir1 ir2 ir3 ir4 ir5 ir6 ir7 ir8 ir9 ira irb cm ct ca ce"
  helifmtstr = 'xx H H H H H  H  x  b x  b x  b B   B   B   B   B   B   B   B   B   B   B   B   H  H  H  H'
  gina2header =    "n p q r ti ta xh xl yh yl zh zl"
  gina2fmtstr = 'xx H H H H H  H  b  b  b  b  b  b'
  openwsnheader =  "n sx sy z1 z3 ta mx my mz lx ly lz ti gx gy gz"
  openwsnfmtstr = '!H H  H  H  H  H  h  h  h  H  H  H  H  h  h  h'
  header =   "n"
  fmtstr = "xxH"
  sport = "COM5"
  chan = 15

  parser = OptionParser(conflict_handler="resolve")
  parser.add_option("-c", "--channel", dest="chan", metavar="INT", help="15.4 channel", default=chan);
  parser.add_option("-p", "--port", dest="sport", metavar="STR", help="Com Port", default=sport);
  parser.add_option("-f", "--format", dest="fmtstr", metavar="STR", help="Format string", default=fmtstr);
  parser.add_option("-h", "--header", dest="header", metavar="STR", help="Header string", default=header);
  parser.add_option("--heli", action="store_true", help="Use helicopter format");
  parser.add_option("--rocket", action="store_true", help="Use rocket format");
  parser.add_option("--gina2", action="store_true", help="Use gina2 format");
  parser.add_option("--openwsn", action="store_true", help="Use openwsn format");
  parser.add_option("--raw", action="store_true", help="output raw bytes");
  (options, args) = parser.parse_args()
  parse = True
  if options.heli:
    (options.fmtstr, options.header) = (helifmtstr, heliheader)
  elif options.rocket:
    (options.fmtstr, options.header) = (rocketfmtstr, rocketheader)
  elif options.gina2:
    (options.fmtstr, options.header) = (gina2fmtstr, gina2header)
  elif options.openwsn:
    (options.fmtstr, options.header) = (openwsnfmtstr, openwsnheader)
  elif options.raw:
    (options.fmtstr, options.header) = (gina2fmtstr, gina2header)
    parse = False

  m = motetalk(options.fmtstr, options.header, options.sport)
  m.sendbase(cmd.radio(int(options.chan)))
  m.sendbase(cmd.mode(cmd.mode_sniff))

  num_good = 0
  num_bad = 0

  done = 0
  n = 0

  log = open("tests/crazywaves26.txt", 'w')
  
  def print8(n):
    return "%8d" % n

  while not done:
    try:
		(arr, t, crcval) = m.nextline(parse)
		print map(ord,arr)
		if (arr == False):
			done = 1
			sys.stderr.write("** Error **\n")
			sys.stderr.write(repr(t) + "\n")
		elif arr:
			if parse:
				print repr(t), repr(arr)
			else:
				arr = map(ord, arr)
				total_length = len(arr)
				print sum(arr)
				if len(arr) > 28:
					if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
						log.write(str("%20.6f"%t))
						log.write(str(arr) + "\n")
						#print "%20.6f" % t, arr
						num_good = num_good + 1
		else:
			num_bad = num_bad + 1
    except:
      sys.stderr.write(repr(sys.exc_info()))
      sys.stderr.write("\nQuitting... \n")
      done = 1

  m.end()

  sys.stderr.write( "%d Successful packets recieved \n" % num_good)
  sys.stderr.write( "%d packet errors \n" % num_bad)
else:
  cmd = ginacmd.ginacmd()
