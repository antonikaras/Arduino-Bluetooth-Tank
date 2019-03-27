#include <TimerOne.h>
#include <Wire.h>
#include <MPU6050.h>
//||---------------------------------------------------------------------------------------------||
//||------------------> Use Accelerometer-Gyroscope to ensure tank orientation <-----------------||
//||---------------------------------------------------------------------------------------------||
//||----------------------------> A P P :: Arduino Bluetooth Tank <------------------------------|| 
//||---------------------------------------------------------------------------------------------||
// ************** E N C O D E R   P I N S ************** //
// ---> motor 1 (right) <--- //
#define enc_1_1 3
#define enc_1_2 12

// ---> motor 2 (left) <---  //
#define enc_2_1 2
#define enc_2_2 11
// ******** M O TO R    D R I V E R    P I N S ******** //
// ---> motor 1 (right) <--- //
#define pwm_1_1 9
#define mot_1_1 4
#define mot_1_2 6

// ---> motor 2 (left) <---  //
#define pwm_2_1 10
#define mot_2_1 7
#define mot_2_2 8
// ********** B L U E T O O T H    P I N S ********** //
// ---> bluetooth <---
#define rxd_pin 1
#define txd_pin 0
//-------------------------------------------------------------------------------------------------
// -----> MPU 6050 variables <----- //
MPU6050 mpu;
/// ---> Timers <--- ///
float timeStep = 0.01;
/// ---> MPU angle data <--- ///
float roll = 0;
float pitch = 0;
float yaw = 0;
/// ---> Orientation Controller Data <--- ///
float tar_orient = 0;
// -----> Encoder 1 variables <----- //
volatile bool Cs1;
volatile bool Ls1;
volatile float cnt1 = 0;
// -----> Encoder 2 variables <----- //
volatile bool Cs2;
volatile bool Ls2;
volatile float cnt2 = 0;
// -----> Controller constants <----- //
int i = 0;
const float Ts = 100;   // Sampling period, millisecond
const float mult = (1000 / Ts) * 60 / 40;
// -----> Controller 1 Data <----- //
float tar_sp_1 = 0;                             // Target Speed
const float kp1 = 300;                          // Kp gain for PID Controller 
const float ki1 = 3000;                         // Ki gain for PID Controller
const float mult1 = (ki1 * (Ts / 1000) - kp1);  // Used for the PID controller, precalculated to speed-up the process
float u1 = 0;                                   // PID controller input
float u_1 = 0;                                  // PID controller previous input
float y_1 = 0;                                  // PID controller previous output
double sum1 = 0;
float mot_1_max_rpm;                            // Maximum rpm for motor 1 (depends on battery voltage)
// -----> Controller 2 Data <----- //
float tar_sp_2 = 0;                             // Target Speed
const float kp2 = 300;                          // Kp gain for PID Controller
const float ki2 = 3000;                         // Ki gain for PID Controller
const float mult2 = (ki2 * (Ts / 1000) - kp2);  // Used for the PID controller, precalculated to speed-up the process
float u2 = 0;                                   // PID controller input
float u_2 = 0;                                  // PID controller previous input
float y_2 = 0;                                  // PID controller previous output
double sum2 = 0;
float mot_2_max_rpm;                            // Maximum rpm for motor 2 (depends on battery voltage)
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
const bool PrintData = false;
bool UseOriantationController = false;           // Use gyroscope data for mor accurate speed calculation
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
#define mapf(x, mi, ma, tmi, tma) (x - mi) * (tma - tmi) / (ma - mi) + tmi    
#define absf(x) ((x > 0) ? (x) : (-x))
#define rpm1_max(x) 0.74811 * x - 106.29975
#define rpm2_max(x) 0.80946 * x - 117.53797
//-------------------------------------------------------------------------------------------------
//*************************************************************************************************
void ENC_1();                                   // Interrupt Service Routine to read Encoder Data for Motor 1
void ENC_2();                                   // Interrupt Service Routine to read Encoder Data for Motor 2
void MotorDriver(int sp1, int sp2);             // Send the speed and direction to each motor
void BluetoothHandler();                        // Connect with the bluetooth device and process data
void SpeedControllers();                        // Calculate the speed for each motor
void MPUGetData();                              // Get data from the gyroscope/accelerometer sensor
void OrientationController();                   // Compute the speed for each motor using a target orientation
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
  /// Use timer to excecute the controller function every Ts
  Timer1.initialize(Ts * 1000);
  Timer1.attachInterrupt(OrientationController); 
  
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

  // sets the maximum milliseconds to wait for serial data
  Serial.setTimeout(5);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {
  // put your main code here, to run repeatedly:

  float time0 = micros();

  BluetoothHandler();
  
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
  Timer1.pwm(pwm_1_1, sp1); 
  
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
  Timer1.pwm(pwm_2_1, sp2); 
}
//-------------------------------------------------------------------------------------------------
// Calculate the speed for each motor
void SpeedControllers()             
{ 
  // Find the maximum rpm for each motor
  float volt = analogRead(A3);
  mot_1_max_rpm = rpm1_max(volt);
  mot_2_max_rpm = rpm2_max(volt);
  
  // -----> M O T O R  1 (right) <----- //
  float rpm1 = mult * cnt1;
  if (rpm1 > 400)
    rpm1 = 0;
 
  cnt1 = 0;
  // ---> Controller 1 <--- //
  float n_tar_sp_1 = mapf(tar_sp_1, 0, 128 , 0, 1);
  float n_rpm_1 = mapf(rpm1, 0, mot_1_max_rpm , 0, 1);

  u1 = absf(n_tar_sp_1) - absf(n_rpm_1);
  float y1 = y_1 + kp1 * u1 + mult1 * u_1;

  // Apply threshold on the speed values [101, 255]
  if (absf(y1) > 255)
    y1 = 255;
  if (tar_sp_1 == 0)
    y1 = 0;
  
  // Store the current input & output values for next time
  y_1 = y1;
  u_1 = u1;    

  // Set motor direction
  if (tar_sp_1 < 0)   
    y1 = -y1;

  sum1 += rpm1;

  // -----> M O T O R  2 <-----
  float rpm2 = mult  * cnt2 ;
  if (rpm2 > 400)
    rpm2 = 0;
  
  cnt2 = 0;
  // ---> Controller 2 <--- //
  float n_tar_sp_2 = mapf(tar_sp_2, 0, 128 , 0, 1);
  float n_rpm2 = mapf(rpm2, 0, mot_2_max_rpm , 0, 1);
 
  u2 = absf(n_tar_sp_2) - absf(n_rpm2);
  float y2 = y_2 + kp2 * u2 + mult2 * u_2;
  
  // Apply threshold on the speed values [101, 255]
  if (absf(y2) > 255)
    y2 = 255;
  if (tar_sp_2 == 0)
    y2 = 0;

  // Store the current input & output values for next time
  y_2 = y2;
  u_2 = u2;
    
  // Set motor direction
  if (tar_sp_2 < 0)   
    y2 = -y2;

  sum2 += rpm2;
  i++;

  MotorDriver(int(4 * y1), int(4 * y2));
  if (PrintData)
  {
    //Serial.print(i);
    Serial.print(" tar_sp_1 ");
    Serial.print(n_tar_sp_1 * mot_1_max_rpm);
    Serial.print(" tar_sp_2 ");
    Serial.print(n_tar_sp_2 * mot_2_max_rpm);
    /*Serial.print(" u1 ");
    Serial.print(u1);
    Serial.print(" u2 ");
    Serial.print(u2);*/
    /*Serial.print(" y1 ");
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
    Serial.println(sum2 / sum1);
   /* Serial.print(" Roll = ");
    Serial.println(roll);
  /*  Serial.print(" Roll = ");
    Serial.print(roll);  
    Serial.print(" Yaw = ");
    Serial.println(yaw);
*/
  }

  // Print orientation data to the app
  Serial.println(roll);
  
}
//-------------------------------------------------------------------------------------------------
// Connect with the bluetooth device and process data
void BluetoothHandler()
{
  String a;                 // stores incoming character from other device
  if (Serial.available())   // if text arrived in from BT serial...
  {
    a=(Serial.readString());
   
    long data = a.toInt();
    Serial.println(data);
    if ((data > 100000) && (data < 500000))
    {
      int rt = ((data % 1000) - 100) - 128;
      int lt = ((data / 1000) - 100) - 128;
      
      if ((absf(rt) > 5) && (absf(rt) < 130))
        tar_sp_1 = -rt;
      else 
        tar_sp_1 = 0;
        
      if ((absf(lt) > 5) && (absf(lt) < 130))
        tar_sp_2 = -lt;
      else 
        tar_sp_2 = 0;  

      if (PrintData)
      {
        Serial.print(data);
        Serial.print(" rt ");
        Serial.print(rt);
        Serial.print(" tar_sp_1 ");
        Serial.print(tar_sp_1);
        Serial.print(" lt ");
        Serial.print(lt);
        Serial.print(" tar_sp_2 ");
        Serial.println(tar_sp_2);
      }
      
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
  if (UseOriantationController)
  {
    if ((tar_orient > -90) && (tar_orient < 90) && (tar_sp_1 != 0) && (tar_sp_2 != 0))
    {
      float error = (tar_orient - roll) / (90 - abs(tar_orient));
      if (error > 0.01)
      {
        tar_sp_1 = max((1 - error), error) * 50 + 200;      // right motor speed
        tar_sp_2 = min((1 - error), error) * 50 + 200;      // left motor speed
      }
      else if (error < -0.01)
      {
        tar_sp_1 = min((1 + error), -error) * 50 + 200;     // right motor speed
        tar_sp_2 = max((1 + error), -error) * 50 + 200;     // left motor speed
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
      
      float error = (tmp_tar_orient - roll) / (90 - tmp_tar_orient);
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

  SpeedControllers();
  
}
