import bluetooth as blt
from inputs import get_gamepad
import time, sys
from termios import tcflush, TCIFLUSH

class Controller(object):

    _lt = -1        # left track
    _rt = -1        # right track
    _orient = -1    # orientation
    _exit = False

    def __init__(self):
        print("Welcome to GamePad Tank Controller")
        print("Searching for nearby bluetooth devices")
        self._nearby_devices = blt.discover_devices(lookup_names=True)

    ###############################################################################################

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

        print("Device :", self._nearby_devices[nb][1], " Connevted Successfully !!!")


    ###############################################################################################

    def ReadGamepadData(self):
        events = get_gamepad()
        for event in events:

            if (event.code == "ABS_Y"):
                self._lt = event.state
                #print("lt", self._lt)

            if (event.code == 'ABS_RZ'):
                self._rt = event.state
                #print("rt", self._rt)

            if (event.code == "BTN_TR2"):
                self._exit = True

    ###############################################################################################

    def SendData(self):

        data = (self._lt + 101) * 1000 + (self._rt + 101)
        
        self._client_socket.send(str(data))
        print(self._lt, self._rt, data)
        time.sleep(5 * 10e-3)           # Sychronize with the Arduino Bluetooth Timeout
        tcflush(sys.stdin, TCIFLUSH)    # Delete gamepad inputs
    ###############################################################################################

    def Disconnect(self):
        self._client_socket.close()

###################################################################################################


cont = Controller()

if (__name__ == '__main__'):
    cont.Connect()
    while not cont._exit:
        cont.ReadGamepadData()
        cont.SendData()

    cont.Disconnect()
