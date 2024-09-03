'''
Title: Read Serial Input from MCU
Author: Matthew Buchkowski 
Date: September 3, 2024

Description:
This purpose of this file is to read in serial data coming in from a specified port. The serial data should be the output of MCU, and in this case if a ATMega328P
sending data read from an adc. 

A 10K thermistor is connected to a EVB and can output an Analog and Digital signal from the peripheral. This file takes the voltage read in at the adc pin, converts 
over to a resistance and then temperature, and finally will display the read temperature as a function of time by continously updating a plot of the values. 

On the plot, both Celcius and Farenhite are displayed. 

NOTE: Further work needs to be done with calibrating the thermistor sensitivity and confirming that post processing of voltage values are correct. 

'''



import serial.tools.list_ports
import matplotlib.pyplot as plt
from math import pow, log
import numpy as np
from time import time

#These values below are assumed for a 10K thermistor and for temperature units of Kelvin
A = 2.1085 * pow(10, -3)
B = 0.79792 * pow(10, -4)
C = 6.53507 * pow(10, -7)

kelvin = 273.15
start_time = time()



def calculateCoeff():
    '''
    This Function will take the hard coded values for the Stien-Hart temperature equation of thermistor resistance at some given (known) temperature (C)
    '''
    A = [[1, log(27513), log(27513) ** 3],
         [1, log(10000), log(10000) ** 3],
         [1, log(4168), log(4168) ** 3] ]
    B = [[0.00],
         [25.0],
         [50.0]]
    return np.linalg.inv(A).dot(B)


def decodPacket(packet):
    """
    Take in the packet of data sent from the MCU/Peripherial, decode, and grab the voltage value
    """
    strData = packet.decode("utf").rstrip('\n')
    voltage = float(strData.split()[-1])
    return voltage

def VoltsToRes(sensedVoltage, refVoltage, thermistorResistance):
    """
    Change the measured sensedVoltage at the ADC input pin and return the appropriate current resistance of the thermistor
    """
    currResistance = sensedVoltage / ( (refVoltage - sensedVoltage) / thermistorResistance)


    return currResistance

def measuredTemperature(currResistance):
    """
    Curing the given, or calculated, coefficients, calculate a temperature
    """

    ans = calculateCoeff()
    # A, B, C = ans[0][0], ans[1][0], ans[2][0]


    tempVal = A + B*log(currResistance) + C*(log(currResistance))**3
    return 1/tempVal


def handleList(valsList, timesList, value):
    '''
    Handle the length of the lists containing the values of data. This will be used for updating the plot of temperatures over time
    '''
    valsList = np.append(valsList, value)
    timesList = np.append(timesList, time() - start_time)

    if len(valsList) >= 21:
        valsList = np.delete(valsList, 0)
        timesList = np.delete(timesList, 0)
    return valsList, timesList

def initPlot():
    '''
    Initialize the plot that will be used for displaying the change in temperature over time
    '''
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax2 = ax.twinx()
    line1, = ax.plot(0, 0, 'r-')
    return fig, ax, line1, ax2


def plotAnalogSignal(timesList, valsList, farenhiteList, fig, lsax, line1):
    '''
    Plot the given values over time
    '''
    ax, ax2 = lsax

    ax.plot(timesList, valsList, 'ro-')
    ax.set_ylim(bottom=np.min(valsList)-3, top=np.max(valsList)+3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (C)")
    ax.set_title("Temperature vs Time of 10k Thermistor")


    ax2.set_ylim(bottom=np.min(farenhiteList)-3, top=np.max(farenhiteList)+3)
    ax2.set_ylabel("Temperature (F)")
 

    fig.canvas.draw()
    fig.canvas.flush_events()

def celciusToFarenhite(cel):
    return (cel * 9/5) + 32

def main():
   
    serialInst = serial.Serial()


    serialInst.baudrate = 115200
    serialInst.port = "/dev/ttyUSB2"
    serialInst.open()
    valsList = np.array([])
    timesList = np.array([])
    farenhiteList = np.array([])

    fig, ax, line1, ax2 = initPlot()

    while True:
        packet = None
        if serialInst.in_waiting:
            packet = serialInst.readline()

        if packet:

            currVoltage = decodPacket(packet)
            
            if currVoltage <= 0.0:
                continue
            currResistance = VoltsToRes(sensedVoltage=currVoltage, refVoltage=5.0, thermistorResistance=10000)

            # print(f"Temperature (K): {measuredTemperature(currResistance)} | Celcius (C) : {measuredTemperature(currResistance) - kelvin}")

            valsList, timesList = handleList(valsList=valsList, timesList=timesList, value=measuredTemperature(currResistance=currResistance) - kelvin)
            farenhiteList, _ = handleList(valsList=farenhiteList, timesList=[], value=celciusToFarenhite(valsList[-1]))
            plotAnalogSignal(timesList, valsList, farenhiteList, fig, [ax, ax2], line1)



    
if __name__ == '__main__':
    main()

