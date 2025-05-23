 
#include <ODriveUART.h>

#include <Wire.h>
#include <Ticker.h>
//#include <SoftwareSerial.h>


#define I2C_SDA 21
#define I2C_SCL 22
// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

#define VDD_sensor_pin 19

float u=0.0, u1=0,u2=0.0, u3=0.0, u4=0.0;
float e=0.0, e1=0.0, e2=0.0, e3=0.0, roll1=0;

// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive ISOVDD and ISOGND to Arduino 3.3V/5V and GND.

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
//SoftwareSerial odrive_serial(8, 9);
//unsigned long baudrate = 19200; // Must match what you configure on the ODrive (see docs for details)

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
// HardwareSerial& odrive_serial = Serial1;
// int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

// Arduino Mega or Due - Serial1
//pin 16: RX - connect to ODrive TX
//pin 17: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial2;
int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)


ODriveUART odrive(odrive_serial);


// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer



// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -2890//-3002//-2306//-6045//-2971//-2624//-883
#define M_Y_MIN -1955//-1560//-1554//-4441//-2094//-2040//+1300
#define M_Z_MIN -3009//-2990//-2605//-5873//-2962//-2852//-2415
#define M_X_MAX +760//+756//+409//+4133//+1169//+696//-745
#define M_Y_MAX +1954//+1556//+1354//+5476//+1847//+1691//+1427
#define M_Z_MAX +738//+603//+129//+4490//+984//+416//-2192

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

//For debugging purposes/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll=0;;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};


Ticker timeri;
const unsigned long interval = 20;  // Intervalo de muestreo en milisegundos <------ min T= 4
const unsigned long intervaltimer = interval; 

unsigned long previousMillis = 0;
float currentMillis=0;

float t=0;
float tref=0;
float tseconds=0;
float trefsegundos=0;



void timerCallback(){
  t=t+intervaltimer;
  tseconds=t/1000.0;

  //tiempo para la referencia 
  tref=tref+intervaltimer;
  trefsegundos=tref/1000.0;

    
  currentMillis=t;

  ODriveFeedback feedback = odrive.getFeedback();


  e=0.0-roll;



  
  //u=  1980947204933732608.0*e1 - 5945122943553126400.0*e2 + 5945085167421375488.0*e3 - 1980909472964295680.0*e + 3.0845805141489108792*u1 - 17993024112573.84375*u2 + 114249980.71484375*u3 + 17992909862591.042969*u4;
  //u=19770076417658.339844*e1 - 19769950795863.484375*e2 + 6587371199687.8388672*e3 - 6587496674624.1123047*ex + 3.8801557573187892558*u1 + 59834494.844342850149*u2 - 376.27573272585868835*u3 - 59834121.44876588136*u4
  //u= 122574473786455.1875*e1 - 122573694931330.39062*e2 + 40841701437058.320312*e3 - 40842479381659.914062*e + 3.9391765155723987846*u1 + 370973897.22023648024*u2 - 2351.7434336543083191*u3 - 370971548.41597932577*u4; //10s
  //u=532.78227731249057797*e1 - 532.71847137722318166*e2 + 176.45038267686780387*e3 - 176.51393710318467356*e + 3.7465061096736462787*u1 - 3.8136504525674346233*u2 + 3.3252825136310186771*u3 - 2.2581381707372303325*u4;
  //u=16.83563840393079758*e2 - 16.814363885180771518*e1 - 5.5838262804970888453*e3 + 5.5625437276747273785*e + 3.9573704705608179211*u1 - 5.9382865386571532795*u2 + 3.9107116676277757783*u3 - 0.92979559953143997575*u4;
  //u=391658555450.66162109*e1 - 391625498822.68902588*e2 + 123495525168.18115234*e3 - 123523755944.00775146*e + 2.751173219597149*u1 + 9386123729.2959747314*u2 - 723091.94561767578125*u3 - 9385400639.1015300751*u4;


  //PI
  u=-(45*e-90*(e-e1));

  //lyapunov
  //u=((26*0.8*9.81*sin(roll)-5*(roll-roll1)/0.2-100*abs((e-e1)/0.2+e)))/16.64;


  if(u>0.8){u=0.8;}if(u<-0.8){u=-0.8;}//limite para que no se desborde u
  odrive.setTorque(u);
  //odrive.setTorque(roll*1.6/0.5235987756);
  
  //Serial.print("pos:");
  //Serial.print(feedback.pos);
  //Serial.print(", ");
  //Serial.print("vel:");
  Serial.print(u);
  Serial.print(", ");
  Serial.print(feedback.vel);
  Serial.print(", ");
  Serial.print(odrive.getParameterAsString("axis0.controller.effective_torque_setpoint"));
  Serial.print(", ");
  printdata();


  u4=u3;
  u3=u2;
  u2=u1;
  u1=u;

  e3=e2;
  e2=e1;
  e1=e;
  roll1=roll;
}

void setup() {
  
  Serial.begin(115200); // Serial to PC
  
  delay(10);

  //Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode (STATUS_LED,OUTPUT);  // Status LED
  pinMode (VDD_sensor_pin,OUTPUT);
  digitalWrite(VDD_sensor_pin,HIGH);

  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;


  odrive_serial.begin(baudrate,SERIAL_8N1,16,17);

  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }

  Serial.println("found ODrive");
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  
  Serial.println("ODrive running!");

  //timer
   timeri.attach_ms(intervaltimer, timerCallback);
}

void loop() {

  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    digitalWrite(STATUS_LED,HIGH);
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;



    // * DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // *

    /*SerialBT.println(" ");
    SerialBT.print(ToDeg(roll));
    SerialBT.print(",");
    SerialBT.print(ToDeg(pitch));
    SerialBT.print(",");
    SerialBT.print(ToDeg(yaw));
    //Serial.print(",");*/
    digitalWrite(STATUS_LED,LOW);
  }
  
}



