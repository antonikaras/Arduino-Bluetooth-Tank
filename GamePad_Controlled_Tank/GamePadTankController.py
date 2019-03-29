#!/usr/bin/python

import bluetooth as blt
from inputs import get_gamepad
import time, sys

###################################################################################################

class GamePadController(object):

    _lt = 128        # left track
    _rt = 128        # right track
    _exit = False

    #---------------------------------------------------------------------------------------------#
   
    # Initiallize class/ bluetooth interface
    def __init__(self):
        print("Welcome to GamePad Tank Controller")
        print("Searching for nearby bluetooth devices")
        self._nearby_devices = blt.discover_devices(lookup_names=True)

    #---------------------------------------------------------------------------------------------#

    # Connect with the HC-05/06 bluetooth module
    def Connect(self):
        print("Choose a device to connect")
        # Print the addresses and names of the nearby devices
        for i in range(0, len(self._nearby_devices)):
            print(i, self._nearby_devices[i])

        # Read device number to connect
        nb = int(input('Choose a number: '))

        # Start socket
        self._client_socket = blt.BluetoothSocket(blt.RFCOMM)

        # Connect to device
        self._client_socket.connect((self._nearby_devices[nb][0], 1))

        print("Device :", self._nearby_devices[nb][1], " Connected Successfully !!!")


    #---------------------------------------------------------------------------------------------#

    def ReadGamepadData(self):

        events = get_gamepad()
        #print(len(events))
        for event in events:
            #print(event.code)
            if (event.code == "ABS_Y"):
                self._lt = event.state
                #print("lt", self._lt)

            if (event.code == 'ABS_RZ'):
                self._rt = event.state
                #print("rt", self._rt)

            if (event.code == "BTN_TR2"):
                self._exit = True

    #---------------------------------------------------------------------------------------------#
    
    # Send 'Compressed' speed data to the arduino
    def SendData(self):

        data = (self._lt + 100) * 1000 + (self._rt + 100)
        print(self._lt, self._rt, data)
   
        self._client_socket.send(str(data))
        #time.sleep(5 * 10e-3)           # Sychronize with the Arduino Bluetooth Timeout
        
    #---------------------------------------------------------------------------------------------#

    # Disconnect from the HC-05/06 module and terminate the app
    def Disconnect(self):
        self._client_socket.close()
    
    #---------------------------------------------------------------------------------------------#

    def Controller(self):
        self.Connect()
        while not self._exit:

            t0 = time.time()
            while (time.time() - t0 < 0.05):
                self.ReadGamepadData()
                #print("time - t0 ?", time.time() - t0)
            self.SendData()


        self.Disconnect()

###################################################################################################

if (__name__ == '__main__'):
    
    cont = GamePadController()
    cont.Controller()
