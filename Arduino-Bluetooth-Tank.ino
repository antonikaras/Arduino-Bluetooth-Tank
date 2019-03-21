//#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
//||---------------------------------------------------------------------------------------------||
//||------------------> Use Accelerometer-Gyroscope to ensure tank orientation <-----------------||
//||---------------------------------------------------------------------------------------------||
//||--------------------------> A P P : ArduinoBluetoothStimpleTank <----------------------------|| 
//||---------------------------------------------------------------------------------------------||
//||--------------------------------> M A X  S P E E D == 4 0 0 <--------------------------------||
//||---------------------------------------------------------------------------------------------||
// ************** E N C O D E R   P I N S ************** //
// ---> motor 1 (right) <---  //
#define enc_1_1 3
#define enc_1_2 11

// ---> motor 2 (left) <---  //
#define enc_2_1 2
#define enc_2_2 10
// ******** M O TO R    D R I V E R    P I N S ******** //
// ---> motor 1 (right) <---  //
#define pwm_1_1 5
#define mot_1_1 4
#define mot_1_2 6

// ---> motor 2 (left) <---  //
#define pwm_2_1 9
#define mot_2_1 7
#define mot_2_2 8

// ********** B L U E T O O T H    P I N S ********** //
// ---> bluetooth <---
#define rxd_pin 1
#define txd_pin 0
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
bool UseOriantationController = false;   // Use gyroscope data for mor accurate speed calculation
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
const float Ts = 100;   // Sampling period, millisecond
const float mult = (1000 / Ts) * 60;
// ---> Controller 1 Data <--- //
float tar_sp_1 = 0;   // Target Speed
const float kp1 = 300;     // Kp gain for PID Controller 19.6569 20 300 21(smooth-slow)
const float ki1 = 2200;      // Ki gain for PID Controller 379.5538 1500 3000 600(smooth-slow)
const float mult1 = (ki1 * (Ts / 1000.) - kp1);
float u1 = 0;           // PID controller input
float u_1 = 0;          // PID controller previous input
float y_1 = 0;          // PID controller previous output
double sum1 = 0;
// ---> Controller 2 Data <--- //
float tar_sp_2 = 0;   // Target Speed
const float kp2 = 300;     // Kp gain for PID Controller
const float ki2 = 1500;    // Ki gain for PID Controller
const float mult2 = (ki2 * (Ts / 1000.) - kp2);
float u2 = 0;           // PID controller input
float u_2 = 0;          // PID controller previous input
float y_2 = 0;          // PID controller previous output
double sum2 = 0;
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
const bool PrintData = true;
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
//SoftwareSerial BT(txd_pin, rxd_pin); 
#define mapf(x, mi, ma, tmi, tma) (x - mi) * (tma - tmi) / (ma - mi) + tmi    
#define absf(x) ((x > 0) ? (x) : (-x))
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
void ENC_1();                         // Interrupt Service Routine to read Encoder Data for Motor 1
void ENC_2();                         // Interrupt Service Routine to read Encoder Data for Motor 2
void MotorDriver(int sp1, int sp2);   // Send the speed and direction to each motor
void BluetoothHandler();              // Connect with the bluetooth device and process data
void SpeedControllers();              // Calculate the speed for each motor
void MPUGetData();                    // Get data from the gyroscope/accelerometer sensor
void OrientationController();         // Compute the speed for each motor using a target orientation
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
void setup()
{
  // put your setup code here, to run once:
    // Use interrupts to read the encoder values
  attachInterrupt (digitalPinToInterrupt (enc_1_1), ENC_1, CHANGE);  // attach interrupt handler
  attachInterrupt (digitalPinToInterrupt (enc_2_1), ENC_2, CHANGE);  // attach interrupt handler
  
  Ls1 = digitalRead(enc_1_1);
  Ls2 = digitalRead(enc_2_1);
  

  // Motor Driver Setup
  /// Motor 1 (right)
  pinMode (mot_1_1, OUTPUT);
  pinMode (mot_1_2, OUTPUT);
  pinMode (pwm_1_1, OUTPUT);
  /// Motor 2 (left)
  pinMode (mot_2_1, OUTPUT);
  pinMode (mot_2_2, OUTPUT);
  pinMode (pwm_2_1, OUTPUT);

  //Serial.begin (115200);
  Serial.begin (9600);

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

  t0 = millis();
  t1 = t0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {
  // put your main code here, to run repeatedly:

  float time0 = micros();

  BluetoothHandler();
  
  if (millis() - t0 > Ts)
  {
    if (UseOriantationController)
      OrientationController();

    SpeedControllers();
    //Serial.println(pitch);
  }

  MPUGetData();
  timeStep = (micros() - time0) / 5.777777778 / 10e6;
}
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
// Interrupt Service Routine to read Encoder Data for Motor 1
void ENC_1()
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Interrupt Service Routine to read Encoder Data for Motor 2
void ENC_2()
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
void MotorDriver(int sp1, int sp2)
{
  
  // Motor 1
  /// Send the Direction to the Driver
  if (sp1 > 0)
  {
      digitalWrite(mot_1_1, HIGH);
      digitalWrite(mot_1_2, LOW);
    }
    else
    {
      digitalWrite(mot_1_1, LOW);
      digitalWrite(mot_1_2, HIGH);
    sp1 = - sp1;
  }

  /// Send the Speed to the Driver
  analogWrite(pwm_1_1, sp1);      
  
  // Motor 2
  /// Send the Direction to the Driver
  if (sp2 > 0)
  {
    digitalWrite(mot_2_1, LOW);
    digitalWrite(mot_2_2, HIGH); 
  }
  else
  {
    digitalWrite(mot_2_1, HIGH);
    digitalWrite(mot_2_2, LOW);
    sp2 = - sp2;
  }

  /// Send the Speed to the Driver
  analogWrite(pwm_2_1, sp2);      
}
//-------------------------------------------------------------------------------------------------
// Calculate the speed for each motor
void SpeedControllers()             
{ 
  // -----> M O T O R  1 (right) <----- //
  float rpm1 = mult * cnt1 / 40 * (millis() - t0) / Ts;
  if (rpm1 > 400)
    rpm1 = 300;
  else if (rpm1 < -400)
    rpm1 = -300;
  
  cnt1 = 0;
  // ---> Controller 1 <--- //
  float n_tar_sp_1 = mapf(tar_sp_1, 0, 300, 0, 1);
  float n_rpm_1 = mapf(rpm1, 0, 300, 0, 1);

  u1 = absf(n_tar_sp_1) - absf(n_rpm_1);
  float y1 = y_1 + kp1 * u1 + mult1 * u_1;

  // Apply threshold on the speed values [101, 255]
  if (absf(y1) > 255)
    y1 = 255;
  if ((absf(y1) < 100) || (tar_sp_1 == 0))
    y1 = 0;
  
  // Store the current input & output values for next time
  y_1 = y1;
  u_1 = u1;    

  // Set motor direction
  if (tar_sp_1 < 0)   
    y1 = -y1;

  sum1 += rpm1;

  // -----> M O T O R  2 <-----
  float rpm2 = mult  * cnt2 / 40 * (millis() - t0) / Ts;
  if (rpm2 > 400)
    rpm2 = 325;
  else if (rpm2 < -400)
    rpm2 = -325;
  
  cnt2 = 0;
  // ---> Controller 2 <--- //
  float n_tar_sp_2 = mapf(tar_sp_2, 0, 325, 0, 1);
  float n_rpm2 = mapf(rpm2, 0, 325, 0, 1);
 
  u2 = absf(n_tar_sp_2) - absf(n_rpm2);
  float y2 = y_2 + kp2 * u2 + mult2 * u_2;
  
  // Apply threshold on the speed values [101, 255]
  if (absf(y2) > 255)
    y2 = 255;
  if ((absf(y2) < 100) || (tar_sp_2 == 0))
    y2 = 0;

  // Store the current input & output values for next time
  y_2 = y2;
  u_2 = u2;
    
  // Set motor direction
  if (tar_sp_2 < 0)   
    y2 = -y2;

  sum2 += rpm2;
  i++;

  MotorDriver(int(y1), int(y2));
  if (PrintData)
  {
    //Serial.print(i);
    Serial.print(" tar_sp_1 ");
    Serial.print(tar_sp_1);
    Serial.print(" tar_sp_2 ");
    Serial.print(tar_sp_2);
    /*Serial.print(" u1 ");
    Serial.print(u1);
    Serial.print(" u2 ");
    Serial.print(u2);*/
    Serial.print(" y1 ");
    Serial.print(y1);
    Serial.print(" y2 ");
    Serial.print(y2);
    /*Serial.print(" rpm1 ");
    Serial.print(rpm1);
    Serial.print(" rpm2 ");
    Serial.print(rpm2);*/
    Serial.print(" rpm1_avg ");
    Serial.print(sum1 / i);
    Serial.print(" rpm2_avg ");
    Serial.print(sum2 / i);
    Serial.print(" rpm2 / rpm1 ");
    Serial.print(sum2 / sum1);
    Serial.print(" Pitch = ");
    Serial.println(pitch);
  /*  Serial.print(" Roll = ");
    Serial.print(roll);  
    Serial.print(" Yaw = ");
    Serial.println(yaw);
*/
  }
  t0 = millis();

}
//-------------------------------------------------------------------------------------------------
// Connect with the bluetooth device and process data
void BluetoothHandler()
{
  String a; // stores incoming character from other device
  if (Serial.available())   // if text arrived in from BT serial...
  {
    a=(Serial.readString());
   
    int data = a.toInt();
    //Serial.println(data);
    
    if (data == 0)    // Stop
    {
      tar_sp_1 = 0;
      tar_sp_2 = 0;
      
      // Reset gyroscope data every time the tank stops
      roll = 0;
      pitch = 0;
      yaw = 0;

      sum1 = 0;
      sum2 = 0;
      i = 0;
    }
    else if (data == 1)   // GO FORWARD
    {
      tar_orient = 0.0;
      tar_sp_1 = 250.0;
      tar_sp_2 = 250.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 2)   // GO FORWARD-RIGHT
    {
      tar_orient = -45.0;
      tar_sp_1 = 200.0;
      tar_sp_2 = 250.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 3)   // ROTATE RIGHT
    {
      tar_orient = 90.0;
      tar_sp_1 = -200.0;
      tar_sp_2 = 200.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 4)   // GO BACKWARD-RIGHT
    {
      tar_orient = 135.0;
      tar_sp_1 = -200.0;
      tar_sp_2 = -250.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 5)   // GO BACKWARD
    {
      tar_orient = 180.0;
      tar_sp_1 = -250.0;
      tar_sp_2 = -250.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 6)   // GO BACKWARD-LEFT
    {
      tar_orient = 225.0;
      tar_sp_1 = -250.0;
      tar_sp_2 = -200.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 7)   // ROTATE LEFT
    {
      tar_orient = 270.0;
      tar_sp_1 = 200.0;
      tar_sp_2 = -200.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else if (data == 8)   // GO FORWARD-LEFT
    {
      tar_orient = 45.0;
      tar_sp_1 = 250.0;
      tar_sp_2 = 200.0;
      sum1 = 0.0;
      sum2 = 0.0;
      i = 0;
    }
    else                 // Deactivate/Activate Orientation Controller
    {
      if (data == 9)
        UseOriantationController = false;
      else if (data == 10)
        UseOriantationController = false;
      tar_sp_1 = 0;
      tar_sp_2 = 0;
      tar_orient =0;
      roll = 0;
      pitch = 0;
      yaw = 0;
      sum1 = 0;
      sum2 = 0;
      i = 0;
    }
  }
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
}
//-------------------------------------------------------------------------------------------------
// Compute the speed for each motor using a target orientation
void OrientationController()
{

  if ((tar_orient > -90) && (tar_orient < 90) && (tar_sp_1 != 0) && (tar_sp_2 != 0))
  {
    float error = (tar_orient - pitch) / (90 - abs(tar_orient));
    if (error > 0.01)
    {
      tar_sp_1 = max((1 - error), error) * 50 + 200;   // right motor speed
      tar_sp_2 = min((1 - error), error) * 50 + 200;   // left motor speed
    }
    else if (error < -0.01)
    {
      tar_sp_1 = min((1 + error), -error) * 50 + 200;  // right motor speed
      tar_sp_2 = max((1 + error), -error) * 50 + 200;  // left motor speed
    }
    else
    {
      tar_sp_1 = 250;
      tar_sp_2 = 250; 
    }
  }

  if ((tar_orient > 90) && (tar_orient < 270) && (tar_sp_1 != 0) && (tar_sp_2 != 0))
  {
    float tmp_tar_orient = tar_orient - 180;
    
    float error = (tmp_tar_orient - pitch) / (90 - tmp_tar_orient);
    if (error > 0.01)
    {
      tar_sp_1 = -(max((1 - error), error) * 50 + 200);   // right motor speed
      tar_sp_2 = -(min((1 - error), error) * 50 + 200);   // left motor speed
    }
    else if (error < -0.01)
    {
      tar_sp_1 = -(min((1 + error), -error) * 50 + 200);  // right motor speed
      tar_sp_2 = -(max((1 + error), -error) * 50 + 200);  // left motor speed
    }
    else
    {
      tar_sp_1 = -250;
      tar_sp_2 = -250; 
    }
  }
}
