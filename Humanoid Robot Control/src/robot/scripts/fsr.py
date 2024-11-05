import serial
import threading
import numpy
import time

class FSR:
    def __init__(self, PORT, BAUD_RATES, TYPE):
        # serial port setting
        self.PORT = PORT
        self.BAUD_RATES = BAUD_RATES
        self.TYPE= TYPE
        self.ser = serial.Serial(self.PORT, self.BAUD_RATES)

        # data preprocessing
        self.F1 = 0
        self.F2 = 0
        self.F3 = 0
        self.F4 = 0
        self.totalF = 0

        # Threading
        self.data_lock = threading.Lock()

    def readData(self):
        self.data_lock.acquire()
        while self.ser.in_waiting:
            data_raw = self.ser.readline()
            data = data_raw.decode()
        
            if "F1: " in data:
                self.F1 = int(data[4:])
            if "F2: " in data:
                self.F2 = int(data[4:])
            if "F3: " in data:
                self.F3 = int(data[4:])
            if "F4: " in data:
                self.F4 = int(data[4:])

            self.totalF = self.F1+self.F2+self.F3+self.F4

            # if self.recordFlag == 1:
            #     recordRow = numpy.array([self.F1, self.F2, self.F3, self.F4])
            #     recordRow = numpy.append([time.time()], recordRow)
            #     self.recordData.append(recordRow)
        self.data_lock.release()
        return numpy.array([self.F1,self.F2,self.F3,self.F4])
        

    # def getZmp(self):
    #     self.zmp_data_lock.acquire()
    #     if self.totalF != 0:
    #         self.zmp[0] = (-self.F1-self.F2+self.F3+self.F4)/self.totalF
    #         self.zmp[1] = (self.F1-self.F2-self.F3+self.F4)/self.totalF
    #     else:
    #         self.zmp[0] = 0
    #         self.zmp[1] = 0
    #     self.zmp_data_lock.release()
    #     return self.zmp, self.totalF

