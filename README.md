# Arduino-Bluetooth-Tank

## Pin-Out
	
	* Arduino Pin 0  <---> HC-06 TXD
     * Arduino Pin 1  <---> HC-06 RXD
	* Arduino Pin 2  <---> Endoder 2 - Data
	* Arduino Pin 3  <---> Endoder 1 - Data
	* Arduino Pin 4  <---> L293D pin 2
	* Arduino Pin 5  <---> 
	* Arduino Pin 6  <---> L293D pin 7
	* Arduino Pin 7  <---> L293D pin 15
	* Arduino Pin 8  <---> L293D pin 10
	* Arduino Pin 9  <---> L293D pin 1
	* Arduino Pin 10 <---> L293D pin 9
	* Arduino Pin 11 <---> Endoder 2 - Clock
	* Arduino Pin 12 <---> Endoder 1 - Clock
	* Arduino Pin 13 <---> 
	* Arduino Pin A4 <---> Voltage devider
	* Arduino Pin A4 <---> GY-521 - SCL
	* Arduino Pin A5 <---> GY-521 - SDA
	
## Robot Description

The robot consists of two 12V DC motors with gears in order to reduce the rpm. The driver used to controlle the motors is a L293D H-Bridge. Using a linear rotary encoder to each motor we can use its values as feedback and create a PI controller. With the help of HC-06 we can connect our mobile phone or PC with the Arduino and controll the vehicle. Finally, using GY-521 we can use the orientation data to correct the vehicles orientation/trajectory.

## Mobile controlled tank

In this project, we used MIT's application in order to create an Android app to controll the tank.

## References

https://www.instructables.com/id/Tutorial-Using-HC06-Bluetooth-to-Serial-Wireless-U/

https://github.com/jarzebski/Arduino-MPU6050

https://www.pjrc.com/teensy/td_libs_TimerOne.html

http://appinventor.mit.edu/explore/
