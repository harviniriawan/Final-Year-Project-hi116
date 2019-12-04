import serial
from datetime import datetime
import time
import io
import threading as th
import subprocess
"""
Script flow:
Create a log textfile

RUN XSCT with the appropriate init_script.tcl, run the device until breakpoint in main

Open serial port, wait until init_script.tcl is finished, then send readFromSerial and
RunProgram thread at the same time

If exit_loop is reached, the task is therefore finished 

"""
#########################################################################################
# Global Variables
#########################################################################################
log_outf    = None      # log file instance
ser         = None      # serial port instance
msg         = ""        # string that buffers messages from 
ser_loop    = 1

def getTime():
  return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

def writeLogs(str_in):
  global log_outf
  print(getTime() + " " + str_in)
  log_outf.write(str_in)
  log_outf.flush()
  return

def resetAndRunDevice():
  writeLogs(getTime() + " [INFO] configuring board...\n")
  ret_value=subprocess.check_call([r'C:\Xilinx\SDK\2019.1\bin\xsct.bat', 'init_script.tcl'])
  return ret_value

def openLogs():
  global log_outf
  global log_dir
  global exec_name
  filemode_inj = 'a'
  log_outf = open('testing.txt', filemode_inj)
  writeLogs(datetime.now().strftime('%Y-%m-%d %H:%M:%S') \
    + " [INFO] starting radiation experiment...\n")
  return

def openSerialInterface():
  global ser
  ser = serial.Serial(port='COM10',baudrate=115200,bytesize=serial.EIGHTBITS,\
    parity=serial.PARITY_NONE,stopbits = serial.STOPBITS_ONE, timeout=5)
  # try:
  #   print("Opening serial port...")
  #   ser.open()
  # except serial.SerialException:
  #   print("something wrong with opening port")
  #   ser.close()
  #   exit()
  return

def readByteFromSerial():
  global ser_loop
  global msg
  d = ser.read(1)

  if (len(d) == 1):
    msg += d.decode('Ascii')
    if (d.decode('Ascii') == '\n'):
      writeLogs(msg)
      msg = ""
  else:
    print("Serial Buffer empty. Some error is happening. Check log")
    ser.close()
    ser_loop = 0
    return

  return

def readSerial():
  while (ser_loop == 1):
    try:
      readByteFromSerial()
    except serial.SerialException:
      print("An serial port exception ocurred...Closing port")
      ser.close()

  print("Out of readSerial while loop...")
  return

def main():
  global ser_loop
  mainLoop = True;
  openLogs()
  while (mainLoop):

    openSerialInterface()
    ret = resetAndRunDevice()
    if (ret != 0):
      writeLogs("[INFO]: Board cannot be reset properly, ending the script\n")
      return 1
    readSerial()
    prompt = input("Would you like to run the test again? y or n ")
    if (prompt == 'y'):
      print ("I see you answered yes")
      mainLoop = True
      ser_loop = 1
    else:
      mainLoop = False
  print("Exiting script...")

main()