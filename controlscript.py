import serial
import sys
import os
from datetime import datetime
import time
import io
import threading as th
import subprocess
import schedule
import pyfirmata
"""
Script flow:
Create a log textfile

RUN XSCT with the appropriate init_script.tcl, run the device until breakpoint in main

Open serial port, wait until init_script.tcl is finished, then send readFromSerial and
RunProgram thread at the same time

If exit_loop is reached, the task is therefore finished 

python sportexample.py logtest.txt split ecc_on pmu_off cacheb01

either ecc_on or ecc_off
either split or lock
either pmu_on or pmu_off
name of kernel is out of the 16 kernels

"""
#########################################################################################
# Global Variables
#########################################################################################
log_outf    = None      # log file instance
ser         = None      # serial port instance
msg         = ""        # string that buffers messages from 
ser_loop    = 1
tcl_script  = 'C:\\Users\\harvi\\Final-Year-Project-hi116\\test_tcl.tcl' # Next time put this in the same directory
logName     = sys.argv[1]
sl_mode     = sys.argv[2]
ecc_mode    = sys.argv[3]
pmu         = sys.argv[4]
kernel      = sys.argv[5]
board       = None

def connect_arduino():
  global board
  board = pyfirmata.Arduino('COM6') # generalise the port next time

def push_reset():
  global board
  board.digital[13].write(0)
  time.sleep(.500)
  board.digital[13].write(1)
  time.sleep(.500)

def push_power(delay):
  global board
  board.digital[12].write(0)
  time.sleep(delay)
  board.digital[12].write(1)
  time.sleep(.500)

def getTime():
  return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

def writeLogs(str_in):
  global log_outf
  print(getTime() + " " + str_in)
  log_outf.write(str_in)
  log_outf.flush()
  return

def manageArgs():
  numArgs = len(sys.argv)
  print(str(sys.argv))
  if numArgs != 6:
    print("Not enough Arguments")

def resetAndRunDevice():
  writeLogs(getTime() + " [INFO] configuring board...\n")
  push_reset()
  ret_value=subprocess.check_call([r'C:\Xilinx\SDK\2019.1\bin\xsct.bat', tcl_script, \
                                  sl_mode, ecc_mode, pmu, kernel])
  print("Ret Value is: " +str(ret_value))
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
  writeLogs(getTime() + "[INFO]: Serial port not receiving any data from host... Resetting the board")
  return

'''Schedule logging of time every few minutes to compare when beam is off'''
def job():
    writeLogs("[INFO] Current Time:  " + getTime() + "\n")

schedule.every(10).minutes.do(job)

def main():
  global ser_loop
  mainLoop = True;
  openLogs()
  connect_arduino();
  push_power(.500);
  while (mainLoop):
    schedule.run_pending()
    # Add another job to change kernel #
    openSerialInterface()
    ret = resetAndRunDevice()
    if (ret != 0):
      writeLogs("[INFO]: Board cannot be reset properly, ending the script\n")
      return 1
    readSerial()
    ser_loop = 1 # Reinit serial loop to 1
  print(getTime()+"[INFO]: Exiting script...")

while (True):
  main()