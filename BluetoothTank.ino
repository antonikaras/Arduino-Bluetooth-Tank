#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
//||---------------------------------------------------------------------------------------------||
//||------------------> Use Accelerometer-Gyroscope to ensure tank orientation <-----------------||
//||---------------------------------------------------------------------------------------------||
//||--------------------------> A P P : ArduinoBluetoothStimpleTank <----------------------------|| 
//||---------------------------------------------------------------------------------------------||
//||--------------------------------> M A X  S P E E D == 2 2 0 <--------------------------------||
//||---------------------------------------------------------------------------------------------||
// ************** E N C O D E R   P I N S ************** // 
// ---> motor 1 (right) <---  //
#define enc_1_1 12  
#define enc_1_2 13

// ---> motor 2 (left) <---  //
#define enc_2_1 10  
#define enc_2_2 11
// ******** M O TO R    D R I V E R    P I N S ******** //
// ---> motor 1 (right) <---  //
#define pwm_1_1 3
#define mot_1_1 2
#define mot_1_2 4

// ---> motor 2 (left) <---  //
#define pwm_2_1 9
#define mot_2_1 7
#define mot_2_2 8 

// ********** B L U E T O O T H    P I N S ********** //
// ---> bluetooth <---
#define rxd_pin 6
#define txd_pin 5
//-------------------------------------------------------------------------------------------------
// ---> MPU 6050 variables <--- //
MPU6050 mpu;
/// ---> Timers <--- ///
unsigned long timer = 0;
float timeStep = 0.01;
/// ---> MPU angle data <--- ///
float roll = 0;
float pitch = 0;
float yaw = 0;
/// ---> Orientation Controller Data <--- ///
float tar_orient = 0;
// ---> Encoder 1 variables <--- //
volatile bool Cs1;
volatile bool Ls1;
volatile float cnt1 = 0;
// ---> Encoder 2 variables <--- //
volatile bool Cs2;
volatile bool Ls2;
volatile float cnt2 = 0;
// ---> Controller constants <--- //
float t0 = -1;
float t1 = -1;
int i = 0;
float Ts = 100;   // Sampling period, millisecond
float mult = (1000 / Ts) * 60;
// ---> Controller 1 Data <--- //
float tar_sp_1 = 0;     // Target Speed
float kp1 = 1.6594;     // Kp gain for PID Controller 1.6594
float ki1 = 23.26;      // Ki gain for PID Controller 23.26
float u1 = 0;           // PID controller input
float u_1 = 0;          // PID controller previous input
float y_1 = 0;          // PID controller previous output
// ---> Controller 2 Data <--- //
float tar_sp_2 = 0;     // Target Speed
float kp2 = 2.5991;     // Kp gain for PID Controller 
float ki2 = 28.0096;    // Ki gain for PID Controller 
float u2 = 0;           // PID controller input
float u_2 = 0;          // PID controller previous input
float y_2 = 0;          // PID controller previous output
// -----> Driver Direction Variables <----- //
bool mot_dir_1_1 = false;
bool mot_dir_1_2 = false;
bool mot_dir_2_1 = false;
bool mot_dir_2_2 = false;
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
SoftwareSerial BT(txd_pin, rxd_pin); 
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
void ENC_1();                         // Interrupt Service Routine For Motor 1
void ENC_2();                         // Interrupt Service Routine For Motor 2
void MotorDriver(int sp, int motID);  // Send the speed and direction to each motor
void BluetoothHandler();              // Connect with the bluetooth device and process data
void SpeedControllers();              // Calculate the speed for each motor
void MPUGetData();                    // Get data from the gyroscope/accelerometer sensor
void OrientationController();         // Compute the speed for each motor using a target orientation
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
void setup() {
  // put your setup code here, to run once:
  
  // Enitiallize encoder  
  Ls1 = digitalRead(enc_1_1);
  Ls2 = digitalRead(enc_2_1);
  t0 = millis();
  
  // Motor Driver Setup
  /// Motor 1
  pinMode (mot_1_1, OUTPUT);
  pinMode (mot_1_2, OUTPUT);
  pinMode (pwm_1_1, OUTPUT);
  /// Motor 2
  pinMode (mot_2_1, OUTPUT);
  pinMode (mot_2_2, OUTPUT);
  pinMode (pwm_2_1, OUTPUT);


  // Define Bluetooth interface
  BT.begin(9600);
  // Send test message to other device
  BT.println("Hello from Arduino Bluetooth Tank");
  
  // Setup MPU6050
  /// Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  /// Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();

  /// Set threshold sensivty. Default 3.
  mpu.setThreshold(3);

  // Setup terminal
  Serial.begin (9600);

  // Store first time stamp
  t1 = t0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop()
{
  // put your main code here, to run repeatedly:
  float time0 = micros();
  
  BluetoothHandler();
  ISR_1();
  ISR_2();
  if (millis() - t0 > Ts)
  {
    OrientationController();
    SpeedControllers();
  }   

  delay(100);
  
  MPUGetData();
  timeStep = (micros() - time0) / 5.777777778 / 10e6;
  
}
//*************************************************************************************************
//-------------------------------------------------------------------------------------------------
// Calculate the speed for each motor
void SpeedControllers()                
{

  // -----> M O T O R  1 <----- //
  float rpm1 = mult * cnt1 / 40 * (millis() - t0) / Ts;
  cnt1 = 0;
  // ---> Controller 1 <--- //
  u1 = abs(tar_sp_1) - abs(rpm1);
  float y1 = y_1 + kp1 * u1 + (ki1 * (Ts / 1000)- kp1)* u_1;
  y_1 = y1;
  u_1 = u1;

  if (tar_sp_1 < 0)
    y1 = -y1;
  if (tar_sp_1 == 0)
    y1 = 0;
  if (y1 < -255)
    y1 = -255 ;
  if (y1 > 255)
    y1 = 255;
    
  // -----> M O T O R  2 <-----
  float rpm2 = mult  * cnt2 / 40 * (millis() - t0) / Ts;
  cnt2 = 0;
  // ---> Controller 2 <--- //
  u2 = abs(tar_sp_2) - abs(rpm2);
  float y2 = y_2 + kp2 * u2 + (ki2 * (Ts / 1000)- kp2)* u_2;
  y_2 = y2;
  u_2 = u2;

  if (tar_sp_2 < 0)
    y2 = -y2;
  if (tar_sp_2 == 0)
    y2 = 0;
  if (y2 < -255)
    y2 = -255 ;
  if (y2 > 255)
    y2 = 255;
/*
  Serial.print(tar_sp_1);
  Serial.print(" ");
  Serial.print(rpm1);
  Serial.print(" - ");
  Serial.print(tar_sp_2);
  Serial.print(" ");
  Serial.println(rpm2);  
  */  
  t0 = millis();
  i++;
  MotorDriver(int(y1), 1);
  MotorDriver(int(y2), 2);
}
//-------------------------------------------------------------------------------------------------
// Interrupt Service Routine For Motor 1
void ISR_1()
{
  Cs1 = digitalRead(enc_1_1);
  if (Cs1 != Ls1)
  {
    if (Cs1 != digitalRead(enc_1_2))
      cnt1++;
    else
      cnt1--;
    Ls1 = Cs1;
  }
}

// Interrupt Service Routine For Motor 2
void ISR_2()
{
  Cs2 = digitalRead(enc_2_1);
  if (Cs2 != Ls2)
  {
    if (Cs2 != digitalRead(enc_2_2))
      cnt2--;
    else
      cnt2++;
    Ls2 = Cs2;
  }
}
//-------------------------------------------------------------------------------------------------
// Send the speed and direction to each motor
void MotorDriver(int sp, int motID)
{
  
  // Motor 2
  if (motID == 1)   
  {
    // Send the Direction to the Driver
    if (sp > 0)
    {
      digitalWrite(mot_1_1, LOW);
      digitalWrite(mot_1_2, HIGH); 
    }
    else
    {
      digitalWrite(mot_1_1, HIGH);
      digitalWrite(mot_1_2, LOW); 
      sp = - sp;
    }

    // Apply speed threshold
    if (sp > 220)
      sp = 220;
    
    // Send the Speed to the Driver
    analogWrite(pwm_1_1, sp);      
  }
  
  // Motor 2
  if (motID == 2)   
  {
    // Send the Direction to the Driver
    if (sp > 0)
    {
      digitalWrite(mot_2_1, LOW);
      digitalWrite(mot_2_2, HIGH); 
       
    }
    else
    {
      digitalWrite(mot_2_1, HIGH);
      digitalWrite(mot_2_2, LOW);
      sp = - sp;
    }
        
    // Apply speed threshold
    if (sp > 220)
      sp = 220;
      
    // Send the Speed to the Driver
    analogWrite(pwm_2_1, sp);      
  }
}
//-------------------------------------------------------------------------------------------------
// Connect with the bluetooth device and process data
void BluetoothHandler()
{
  String a; // stores incoming character from other device
  if (BT.available())   // if text arrived in from BT serial...
  {
    a=(BT.readString());
    Serial.println(a);
    if (a=='?')
    {
      BT.println("Send '1' to turn LED on");
      BT.println("Send '2' to turn LED on");
    }   
    else
    {
      int data = a.toInt();
      Serial.println(a.toInt());
      
      if (data == 0)    // Stop
      {
        tar_sp_1 = 0;
        tar_sp_2 = 0;
        
        // Reset gyroscope data every time the tank stops
        roll = 0;
        pitch = 0;
        yaw = 0;
      }
      else if (data == 1)   // GO FORWARD
      {
        tar_orient = 0;
        tar_sp_1 = 210;
        tar_sp_2 = 210;
      }
      else if (data == 2)   // GO FORWARD-RIGHT
      {
        tar_orient = 45;
        tar_sp_1 = 190;
        tar_sp_2 = 210;
      }
      else if (data == 3)   // ROTATE RIGHT
      {
        tar_orient = 90;
        tar_sp_1 = -190;
        tar_sp_2 = 190;
      }
      else if (data == 4)   // GO BACKWARD-RIGHT
      {
        tar_orient = 135;
        tar_sp_1 = -190;
        tar_sp_2 = -210;
      }
      else if (data == 5)   // GO BACKWARD
      {
        tar_orient = 180;
        tar_sp_1 = -210;
        tar_sp_2 = -210;
      }
      else if (data == 6)   // GO BACKWARD-LEFT
      {
        tar_orient = 225;
        tar_sp_1 = -210;
        tar_sp_2 = -190;
      }
      else if (data == 7)   // ROTATE LEFT
      {
        tar_orient = 270;
        tar_sp_1 = 190;
        tar_sp_2 = -190;
      }
      else if (data == 8)   // GO FORWARD-LEFT
      {
        tar_orient = 315;
        tar_sp_1 = 210;
        tar_sp_2 = 190;
      }
    }
  }
    // you can add more "if" statements with other characters to add more commands
}
//-------------------------------------------------------------------------------------------------
// Get data from the gyroscope/accelerometer sensor
void MPUGetData()
{
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep * 180 / 3.1415926;
  roll = roll + norm.XAxis * timeStep * 180 / 3.1415926;
  yaw = yaw + norm.ZAxis * timeStep * 180 / 3.1415926;

  
  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);
  
}
//-------------------------------------------------------------------------------------------------
// Compute the speed for each motor using a target orientation
void OrientationController()
{
  
  if (tar_orient > 270)
    tar_orient -= 360;

  if ((tar_orient > -90) && (tar_orient < 90) && (tar_sp_1 != 0) && (tar_sp_2 != 0))
  {
    float error = (tar_orient - pitch) / (90 - abs(tar_orient));
    if (error > 0.05)
    {
      tar_sp_1 = (1 - error) * 40 + 180;                // right motor speed
      tar_sp_2 = error * 40 + 180;                      // left motor speed
      Serial.print(" error > 0.05 ");
    }
    else if (error < -0.05)
    {
      tar_sp_1 = -error * 40 + 180;                     // right motor speed
      tar_sp_2 = (1 - error) * 40 + 180;                // left motor speed
      Serial.print(" error < -0.05 ");
      Serial.print(abs(error));
    }

    Serial.print(" tar_orient ");
    Serial.print(tar_orient);
    Serial.print(" error ");
    Serial.print(error);
    Serial.print(" tar_sp_1 ");
    Serial.print(tar_sp_1);
    Serial.print(" tar_sp_2 ");
    Serial.println(tar_sp_2);
    
  }

  if ((tar_orient > 90) && (tar_orient < 270) && (tar_sp_1 != 0) && (tar_sp_2 != 0))
  {
    float tmp_tar_orient = tar_orient - 180;
    
    float error = (tmp_tar_orient - pitch) / (90 - tmp_tar_orient);
    if (error > 0.05)
    {
      tar_sp_1 = -(1 - error) * 40 + 180;               // right motor speed
      tar_sp_2 = -error * 40 + 180;                     // left motor speed
    }
    else if (error < -0.05)
    {
      tar_sp_1 = -error * 40 + 180;                     // right motor speed
      tar_sp_2 = -(1 - error) * 40 + 180;               // left motor speed
    }

    Serial.print(" tar_orient ");
    Serial.print(tar_orient);
    Serial.print(" error ");
    Serial.print(error);
    Serial.print(" tar_sp_1 ");
    Serial.print(tar_sp_1);
    Serial.print(" tar_sp_2 ");
    Serial.println(tar_sp_2);
    
  }
  
    
}
