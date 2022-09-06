from math import e
import time
from cv2 import data
import serial
from time import process_time_ns, sleep
import numpy as np
import cv2
from sys import argv
from array import array
import struct
import scipy.io as scio
import sys
import keyboard

np.set_printoptions(linewidth=np.inf)
INIT_FILE = input("Enter paramter set value: ")
OUT_FILE = input("Enter trials name: ") 
suffix = input("Enter the trials number: ")
# RS232Tx/RX is for config port
# AR_mss_logger is for data port

# hb = find_sublist(MW, res) - 1


def find_sublist(sub, bigger):
    # sub -> MW, bigger -> res readline()
    # MW = [2, 1, 4, 3, 6, 5, 8, 7]
    first, rest = sub[0], sub[1:]
    # 2，1， 4，3，6，5，8，7
    pos = 0
    try:
        while True:
            # index = alphabets.index('i',4)
            # i after 4th index
            pos = bigger.index(first, pos) + 1
            # index of 'first' after 'pos' position
            # For the current input, position of 2 after the perviously found
            if not rest or bigger[pos:pos+len(rest)] == rest:
                # if the rest is not empty or
                # the readline[pos:pos+length(rest)] == rest
                return pos
            # the postion of MW.
    except ValueError:
        return -1


def TTY_INIT(tty, speed):
    ser = serial.Serial()
    ser.port = tty  # "/dev/ttyUSB0" #ttyACM0"
    ser.baudrate = speed
    ser.NewLine = "\n"
    ser.open()
    return ser


def Radar_INIT(serial):
    log = open(INIT_FILE, "r")
    while True:
        line = log.readline()
        # count the row of the file.
        lenght = len(line)
        if(lenght > 9):
            if line[10] is not '%' and line[0:10] == 'mmwDemo:/>':
                # read the command
                serial.write(line[10:lenght].encode("utf-8")+b'\n')

                t1 = time.perf_counter()
                while True:
                    res = serial.readline()
                    print(res)
                    if res == b'Done\n':
                        break
                    sleep(0.05)
                    # break
                    if time.perf_counter() > t1 + 0.3:  # wait 1 sec
                        serial.write(line[10:lenght].encode(
                            "utf-8") + b'\n')  # repeat sending
                        t1 = time.perf_counter()

        if line[0] == 'q':
            print("END OF FILE")
            break
    log.close()


def Radar_START(serial):
    sleep(0.1)
    serial.write('sensorStart'.encode("utf-8") + b'\n')
    # while True:
    #    res = serial.readline()
    #    print(res)
    #    if res == b'Done\n':
    #        break
    #    sleep(0.05)
    print("RADAR STARTED")


def Radar_STOP(serial):
    sleep(0.1)
    serial.write('sensorStop'.encode("utf-8") + b'\n')
    # while True:
    #    res = serial.readline()
    #    print(res)
    #    if res == b'Done\n':
    #        break
    #    sleep(0.05)
    print("RADAR STOPPED")


# Display image
def Radar_window(frame_int):
    # frame[,,:] = (24, 27, 68)
    # frame = np.zeros((480, 640, 3), np.uint8)
    # Draw a line on graph
    #        image      start point  end_point  color    thinkness
    cv2.line(frame_int, (320, 0), (320, 470), (0, 0, 255), 1)  # (24, 27, 68)
    cv2.line(frame_int, (0, 470), (640, 470), (0, 0, 255), 1)  # (24, 27, 68)
    #cv2.line(frame, (zone_int[4], zone_int[5]), (zone_int[6], zone_int[7]), (0, 255, 0), 2)
    # show the output frame
    # frame_int = cv2.resize(frame_int, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)  # x2 image
    cv2.imshow("Frame", frame_int)


def Radar_add_points(frame_int, buffer):
    #           image     tuple contain x,      y coordinate         radius       color  thinkness
    cv2.circle(frame_int, (320 + buffer[3]*20,
               470 - buffer[4]*20), 4, (0, 255, 0), -1)
    return frame_int


def converter(data, local_counter, data_stage):
    try:
        if(data_stage == 0):
            data = data[local_counter]
            #print("-------")
           #print("Number of Frame: ", data)
        if(data_stage == 1):
            data = data[local_counter]
            #print("Number of object ",data)
        if(data_stage == 2):
            data = data[local_counter]
            #print("Length of payload:", data)
        if(data_stage == 3):
            data = data[local_counter]
            #print("Format: ",data)
            #print("--------------")
        # if(data_stage>=4):
        #     #print("data: ",data)
        #     data = (data[local_counter:])
        #     #print("data1: ",data)
        #     data = data[:size]
        #     #print("data2: ",data)
            
        if (data_stage == 4) :
            data = struct.unpack('<H', bytes(data))
            data = "%d" % data
            #print("Range index: ", int(data))
        if (data_stage == 5) :
            data = struct.unpack('<h', bytes(data))
            data = "%d" % data
            #print("Doppler index: ", int(data))
        if (data_stage == 6) :
            data = struct.unpack('<H', bytes(data))
            data = "%d" % data
            #print("Peak Value: ", int(data))
        if (data_stage == 7) :
            data = struct.unpack('<h', bytes(data))
            data = "%d" %(data)
            #print("X: ", int(data))
        if (data_stage == 8) :
            data = struct.unpack('<h', bytes(data))
            data = "%d" %(data)
            #print("Y: ", int(data))
        if (data_stage == 9) :
            data = struct.unpack('<h', bytes(data))
            data = "%d" %(data)
            #print("Z: ", int(data))
        return int(data)
    except struct.error as e:
        print("unexpected: ",e)
# ----------------------------------------------------------------------------------------------------------------------------


def Radar_READ_DATA(serial):
    # time mark
    T1 = time.time()
    MW = [2, 1, 4, 3, 6, 5, 8, 7]
    HEAD_INDEX = [20, 28, 40, 46]
    # Number of Frame
    # Number of detedted objects
    # Length of structure
    # Descriptor: detected obj num, xyz format
    MAIN_HEAD = 48
    # [0.0, 0.0, 0.0, 0.0]
    # The HEAD in the DATA
    DATA_HEAD = list(np.zeros(4))
    # array []
    DATA_MAIN = np.zeros((6), dtype='int16')
    XYZ_Cor = np.zeros((5), dtype = 'float')
    DATA_BUFFER = [0,0]
    local_counter = 0
    corDic = {'frame':[],'x':[],'y':[],'z':[]}
    MW_marker = 0  # number of beginning of the header in the packet
    global_counter = 0
    data_stage = 0
    num_Coun = 0
    frame = np.zeros((480, 640, 3), np.uint8)
    # perf_counter retuen time in seconds. -> 6000s -> 100min
    # while time.perf_counter() < t1+6000: #wait 5 sec
    while True:
        res = list(serial.readline())
        lenght = len(res)
        print("packet_Length:", lenght)
        MW_marker = find_sublist(MW, res) - 1  # find place of array
        # The position of the first MW
        print("position of the first MW:", MW_marker)
        local_counter = 0
        # The current reading bit is part of the incoming stream
        while local_counter < lenght:
            #if MW_marker >= 0:
            # if read through an magic world, new packet detected 
            if local_counter == MW_marker:
                global_counter = 0  # set to beginning of the header - new packet
                data_stage = 0  # new packet with new header
                #Radar_window(frame)  # show radar data on the screen
                #frame = np.zeros((480, 640, 3), np.uint8)
                print("---One start---")
            if data_stage < 4:
                if global_counter == HEAD_INDEX[data_stage]:
                    # return context
                    DATA_HEAD[data_stage] = converter(res, local_counter, data_stage)
                    data_stage += 1
                    if data_stage == 4:
                        num_Coun = 0
                        print("data_HEAD: ", DATA_HEAD)

            else:
                try:
                    if global_counter >= 48 and global_counter < 45 + (DATA_HEAD[1] * 12 + 4):
                        if data_stage == 10:
                            num_Coun+=1
                            #print("data_Main: ", DATA_MAIN)
                            XYZ_Cor[0] = DATA_HEAD[0]
                            XYZ_Cor[1] = num_Coun
                            XYZ_Cor[2] = DATA_MAIN[3] /(1 << DATA_HEAD[3])
                            XYZ_Cor[3] = DATA_MAIN[4] /(1 << DATA_HEAD[3])
                            XYZ_Cor[4] = DATA_MAIN[5] /(1 << DATA_HEAD[3])
                            #frame = Radar_add_points(frame, DATA_MAIN)
                            print("XYZ_Cor: ", XYZ_Cor)
                            corDic.get('frame').append(XYZ_Cor[0])
                            corDic.get('x').append(XYZ_Cor[2])
                            corDic.get('y').append(XYZ_Cor[3])
                            corDic.get('z').append(XYZ_Cor[4])
                            data_stage = 4
                        elif global_counter % 2 == 0:
                            DATA_BUFFER[0] = res[local_counter]
                        else:
                            DATA_BUFFER[1] = res[local_counter]
                            DATA_MAIN[data_stage-4] = converter(DATA_BUFFER, local_counter, data_stage)
                            data_stage+=1
                except Exception as e:
                    print("caught error!",e)
                    print("data_stage: ",data_stage)
                    print("data_main: ",DATA_MAIN)
                    break
            global_counter += 1
            local_counter += 1

        if keyboard.is_pressed('Esc'):
            #print("process end")
            T2 = time.time()
            dataFile = OUT_FILE+'_'+suffix+'.mat'
            scio.savemat(dataFile,corDic)
            print("Running time: %s s"%(T2-T1))

            break
            # Data from last line
    print("Time is passed")


def Main():
    # open the port
    ser = TTY_INIT("COM7", 115200)

    # config
    Radar_INIT(ser)
    Radar_START(ser)
    
    ser = TTY_INIT("COM6", 921600)
    Radar_READ_DATA(ser)
    ser = TTY_INIT("COM7", 115200)
    Radar_STOP(ser)


Main()
quit()

# add to the end of config file if only dynamic points needed
# mmwDemo:/>clutterRemoval 1
# Done
