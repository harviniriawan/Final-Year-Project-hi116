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
import win32com.client as win32
from tempfile import mkstemp
from shutil import move, copymode
from os import fdopen, remove
"""
Script flow:
Create a log textfile

RUN XSCT with the appropriate init_script.tcl, run the device until breakpoint in main

Open serial port, wait until init_script.tcl is finished, then send readFromSerial and
RunProgram thread at the same time

If exit_loop is reached, the task is therefore finished 

python controlscript.py no_ecc_feb.txt split ecc_off pmu_off

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
tcl_script  = 'C:\\Users\\harvi\\Final-Year-Project-hi116\\tcm_test_tcl.tcl'
A53_inj_p   = 'C:\\Users\\harvi\\Final-Year-Project-hi116\\A53_INJ\\src\\A53_injector.c'
logName     = sys.argv[1]
sl_mode     = sys.argv[2]
ecc_mode    = sys.argv[3]
pmu         = sys.argv[4]
board       = None
rseed       = 0
kernels_end = ['0x4b74', '0x3dec', '0x5100', '0x4ca0',\
               '0x4748','0x3d2c','0xa578',\
               '0x49cc','0x5a1c','0x5b18','0x6918','0x3f50',\
               '0x46a0','0x3a70','0x42b8','0x5e98']
kernels     = ['a2time01','canrdr01','aifftr01','aiifft01',\
               'aifirf01', 'basefp01', 'bitmnp01',\
               'cacheb01', 'idctrn01','iirflt01', 'matrix01', 'pntrch01',\
               'puwmod01', 'rspeed01', 'tblook01','ttsprk01']

curr_kernel = 0

num_reset = 0


def connect_arduino():
  global board
  board = pyfirmata.Arduino('COM7') # generalise the port next time

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
  global rseed
  global kernels
  global curr_kernel
  writeLogs(getTime() + " [INFO] configuring board...\n")
  push_reset()
  writeLogs(getTime() + "Current seed : " + str(rseed) + "\n")
  replace(A53_inj_p,'srand(' + str(rseed),'srand('+ str(rseed+1))
  ret_value=subprocess.check_call([r'C:\Xilinx\SDK\2019.1\bin\xsct.bat', tcl_script, \
                                  sl_mode, ecc_mode, pmu, kernels[curr_kernel]])
  print("Ret Value is: " +str(ret_value))
  rseed = rseed + 1
  writeLogs(getTime() + " Increment seed to : " + str(rseed) + "\n")
  return ret_value

def openLogs(lname):
  global log_outf
  global log_dir
  global exec_name
  filemode_inj = 'a'
  log_outf = open(lname+'.txt', filemode_inj)
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
  global num_reset
  try:
    d = ser.read(1)
    if (len(d) == 1):
      msg += d.decode('utf-8')
      if (d.decode('utf-8') == '\n'):
        writeLogs(msg)
        print("Number of resets in this kernel: " + str(num_reset))
        msg = ""
    else:
      print("Serial Buffer empty. Some error is happening. Check log")
      ser.close()
      ser_loop = 0
      return
  except serial.SerialException:
    writeLogs(getTime()+ " [ERROR]: Corruption happening while reading a byte from serial port...Resetting\n")
    ser.close()
  return

def readSerial():
  global ser_loop
  global num_reset
  while (ser_loop == 1):
    try:
      schedule.run_pending()   # Check if kernel needs to be changed
      readByteFromSerial()
    except serial.SerialException:
      writeLogs(getTime()+ " [ERROR]: A serial port exception ocurred...Closing port\n")
      ser.close()

  print(getTime() + " Out of readSerial while loop...")
  num_reset = num_reset + 1 # Find out roughly how many resets has occurred
  writeLogs(getTime() + " [INFO]: Serial port not receiving any data from host... Resetting the board\n")
  return

'''Schedule logging of time every few minutes to compare when beam is off'''
def logtime():
    writeLogs(getTime() + " [INFO] Current Time  " + "\n")

'''Change kernel'''
def changeKernel():
  global curr_kernel
  global kernels
  global ser
  global num_reset
  global log_outf
  ser.flush() # Flush serial port before changing kernel
  ser.reset_input_buffer()
  ser.reset_output_buffer()
  writeLogs(getTime() + " [INFO] Change kernel " + "\n")
  push_reset()
  writeLogs(getTime() + " [INFO] Kernel before change :" + kernels[curr_kernel] + "\n")
  # First pass will always increment kernel pointer
  if curr_kernel == 0 :
    writeLogs(getTime() + " [INFO] Increment Kernel pointer...\n")
    curr_kernel = curr_kernel+1
  elif (curr_kernel % 15) != 0 :
    writeLogs(getTime() + " [INFO] Increment Kernel pointer...\n")
    curr_kernel = curr_kernel+1
  else:
    writeLogs(getTime() + " [INFO] Kernel pointer set back to 0...\n")
    curr_kernel = 0

  # Close current log file
  log_outf.close()
  # Open new log for new benchmark
  openLogs(kernels[curr_kernel])

  num_reset = -1 # Reinitialise number of reset to -1, as it will reset once
  #Change endpoint of kernel
  replace(A53_inj_p,'#define BENCH_END '+ kernels_end[curr_kernel-1],'#define BENCH_END '+ kernels_end[curr_kernel])
  writeLogs(getTime() + " [INFO] Current kernel  : " + kernels[curr_kernel] + "\n")

schedule.every(1).minutes.do(logtime)
schedule.every(3).minutes.do(changeKernel)

def send_email():
  global num_reset
  outlook = win32.Dispatch('outlook.application')
  mail = outlook.CreateItem(0)
  mail.To = 'harvin.iriawan4534@gmail.com'
  mail.Subject = str(num_reset)
  mail.Body = 'Message body'
  mail.HTMLBody = '<h2>Get to the lab, power cycling Needed</h2>' #this field is optional
  mail.Send()

def replace(file_path, pattern, subst):
  #Create temp file
  fh, abs_path = mkstemp()
  with fdopen(fh,'w') as new_file:
      with open(file_path) as old_file:
          for line in old_file:
              new_file.write(line.replace(pattern, subst))
  #Copy the file permissions from the old file to the new file
  copymode(file_path, abs_path)
  #Remove original file
  remove(file_path)
  #Move new file
  move(abs_path, file_path)



def main():
  global ser_loop
  global num_reset
  mainLoop = True;
  openLogs(kernels[curr_kernel])
  connect_arduino();
  push_power(.500);
  while (mainLoop):
    schedule.run_pending()
    openSerialInterface()
    ret = resetAndRunDevice()
    if (ret != 0):
      writeLogs(getTime()+" [INFO]: Board cannot be reset properly, ending the script\n")
      mainLoop = False
    readSerial()
    if (num_reset >= 10):
      print("Lots of error\n")
    ser_loop = 1 # Reinit serial loop to 1
  print(getTime()+" [INFO]: Exiting script...\n")

print(os.path.abspath(__file__))
# main()
